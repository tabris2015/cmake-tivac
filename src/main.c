// #define PART_TM4C1230C3PM
#define UART_BUFFERED
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h> //* abs function
#include <math.h>
#include <inc/tm4c123gh6pm.h>
#include <inc/hw_gpio.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>

#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/interrupt.h>
#include <driverlib/gpio.h>
#include <driverlib/pwm.h>
#include <driverlib/qei.h>
#include <driverlib/timer.h>
#include <driverlib/systick.h>
#include <driverlib/uart.h>
#include <driverlib/adc.h>

#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

#include "board_defines.h"

#include "pid.h"

typedef struct robot_state_t
{
    float l_setpoint;
    float r_setpoint;
    uint32_t l_ticks;
    uint32_t r_ticks;
    float l_angle;
    float r_angle;
    float l_speed;
    float r_speed;
}robot_state_t;

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define APP_INPUT_BUF_SIZE  64
#define SYSTICK_PERIOD 79999

//* globals
//* for control loop
volatile bool c_flag = false;
//* for timing
volatile uint32_t millisecs = 0;

//* for motor control
volatile int qei_pos;

volatile int motor1_ticks = 0;

//* for leds
volatile uint32_t led_pwm = 0;
volatile uint32_t red_pwm = 0;
volatile uint32_t green_pwm = 0;

volatile uint8_t state = 0;


//* for cmdline
static char g_cInput[APP_INPUT_BUF_SIZE];


//* timing functions
void delayMS(int ms)
{
    SysCtlDelay((SysCtlClockGet() / (3 * 1000)) * ms);
}
void SystickCB()
{
    millisecs++;
}
//* prototypes
void InitCLock()
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN); //  80Mhz
    // SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);  //  16Mhz
}

void InitBoard();
void InitLED();
void InitLeftMotor();
void InitRightMotor();
void InitEncoder1();
void InitEncoder2();
void InitSystick();
void InitTimer0();
void InitConsole();
void InitTemp();


//* prototypes for motors api
void lmotor(int vel);
void rmotor(int vel);

//* prototypes for encoder
uint32_t getLMotorTicks();
int32_t getLMotorVel();
uint32_t getRMotorTicks();
int32_t getRMotorVel();
//* led api
void setLED(uint32_t red, uint32_t green);

//* timer callback
void loopISR();

//* adc
uint32_t readTempC();

//* serial command
void serialCommand(char * buffer);
//* for pid
void initPids(PidData_t *left, PidData_t *right);

int main(void)
{
    InitBoard();
    delayMS(100);
    setLED(1, 1);
    delayMS(100);
    uint8_t green_state = 0;
    uint32_t last_green = millisecs;
    uint32_t last_log = millisecs;
    int32_t i32CommandStatus;
    char buffer[10];
    float v_setpoint = 0.0f;
    int log_vel = 20;
    bool log_flag = true;
    lmotor(0);
    rmotor(0);
    robot_state_t robot_state;

    PidData_t l_pid_data;
    PidData_t r_pid_data;

    float left_speed;
    float left_pwm;
    float left_setpoint = 0;
    float right_speed;
    float right_pwm;
    float right_setpoint = 0;
    
    
    l_pid_data.input_ptr = &left_speed;
    l_pid_data.output_ptr = &left_pwm;
    l_pid_data.setpoint_ptr = &left_setpoint;
    l_pid_data.sample_time_ms = DELTA_T_MS;
    l_pid_data.out_sum = 0;
    l_pid_data.last_input = 0;
    PidSetOutputLimits(&l_pid_data, MIN_PWM, MAX_PWM);
    PidSetGains(&l_pid_data, 1.0, 0.0, 0.1);

    r_pid_data.input_ptr = &right_speed;
    r_pid_data.output_ptr = &right_pwm;
    r_pid_data.setpoint_ptr = &right_setpoint;
    r_pid_data.sample_time_ms = DELTA_T_MS;
    r_pid_data.out_sum = 0;
    r_pid_data.last_input = 0;
    PidSetOutputLimits(&r_pid_data, MIN_PWM, MAX_PWM);
    PidSetGains(&r_pid_data, 1.0, 0.0, 0.1);

    
    while (1)
    {
        if(UARTPeek('\r') != -1)
        {
            char * p_end;
            //* new command received
            UARTgets(g_cInput,sizeof(g_cInput));

            //* process command]
            switch (g_cInput[0])
            {
            case 'q':
                log_flag = false;
                break;
            case 'v':
                log_flag = true;
                break;
            case 'w':
                //! read setpoints [rad/s]
                left_setpoint = ustrtof(g_cInput + 1, &p_end);
                right_setpoint = ustrtof(p_end, NULL);
                break;

            default:
                break;
            }
        }
        
        if(c_flag)
        {
            //* control loop calculations
            c_flag = false;
            
            //* read inputs
            int32_t l_delta_ticks = getLMotorVel();
            int32_t r_delta_ticks = getRMotorVel();
            float l_delta_phi = 2* M_PI * (l_delta_ticks / ENC_PPR);            //* [rad]
            left_speed = l_delta_phi / (DELTA_T_MS / 1000.0);                   //* [rad/s]
            float r_delta_phi = 2* M_PI * (r_delta_ticks / ENC_PPR);            //* [rad]
            right_speed = r_delta_phi / (DELTA_T_MS / 1000.0);                  //* [rad/s]

            //* compute pid
            PidCompute(&l_pid_data);
            PidCompute(&r_pid_data);
            
            //* out
            lmotor((int)left_pwm);
            rmotor((int)right_pwm);
        }
        uint32_t current = millisecs;
        if(current - last_green > 1000)
        {
            green_pwm = green_state ? 1 : 200;
            setLED(red_pwm, green_pwm);
            green_state = !green_state;

            last_green = millisecs;
        }
        if((current - last_log >= DELTA_T_MS) && log_flag)
        {
            UARTprintf("%d %d %d; %d %d \n", millisecs, getLMotorTicks(), getRMotorTicks(), getLMotorVel(), getRMotorVel());
            last_log = millisecs;
        }

    }
}

//* functions
void InitBoard()
{
    InitCLock();
    InitLED();
    InitEncoder1();
    InitEncoder2();
    InitLeftMotor();
    InitRightMotor();
    InitConsole();
    InitTemp();
    InitTimer0();
    InitSystick();
    IntMasterEnable();
    UARTprintf("initialization finished!\n");

}

void InitLED()
{
    SysCtlPeripheralEnable(LED_PERIPH);
    SysCtlPeripheralEnable(LED_PWM_PERIPH);
    //red led
    GPIOPinConfigure(RED_LED_PINMUX);
    GPIOPinConfigure(GREEN_LED_PINMUX);
    
    GPIOPinTypePWM(LED_PORT, RED_LED_PIN);
    GPIOPinTypePWM(LED_PORT, GREEN_LED_PIN);
    
    PWMGenConfigure(LED_PWM_BASE, RED_LED_GEN, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(LED_PWM_BASE, GREEN_LED_GEN, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(LED_PWM_BASE, RED_LED_GEN, M_PWM_PERIOD);
    PWMGenPeriodSet(LED_PWM_BASE, GREEN_LED_GEN, M_PWM_PERIOD);
    
    PWMPulseWidthSet(LED_PWM_BASE, RED_LED_GEN, 1);
    PWMPulseWidthSet(LED_PWM_BASE, GREEN_LED_GEN, 1);

    PWMGenEnable(LED_PWM_BASE, RED_LED_GEN);
    PWMGenEnable(LED_PWM_BASE, GREEN_LED_GEN);
    
    PWMOutputState(LED_PWM_BASE, RED_LED_OUT_BIT | GREEN_LED_OUT_BIT, true);
    
    // GPIOPinWrite(RED_LED_PORT, RED_LED_PIN, 0);
}

void InitLeftMotor()
{
    //* Digital pins for direction
    SysCtlPeripheralEnable(M1_IN_PERIPH);
    GPIOPinTypeGPIOOutput(M1_PORT, M1_IN1_PIN | M1_IN2_PIN);
    GPIOPinWrite(M1_PORT, M1_IN1_PIN | M1_IN2_PIN, 0);

    //* Pwm signals
    SysCtlPeripheralEnable(M1_PWM_PERIPH);
    GPIOPinConfigure(M1_PWM_PINMUX); // m1
    GPIOPinTypePWM(M1_PORT, M1_PWM_PIN);
    PWMGenConfigure(M1_PWM_BASE, M1_PWM_GEN, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    //* period
    PWMGenPeriodSet(M1_PWM_BASE, M1_PWM_GEN, M_PWM_PERIOD);
    PWMPulseWidthSet(M1_PWM_BASE, M1_PWM_OUT, 1);

    PWMGenEnable(M1_PWM_BASE, M1_PWM_GEN);

    PWMOutputState(M1_PWM_BASE, M1_PWM_OUT_BIT, true);
}

void InitRightMotor()
{
    //* Digital pins for direction
    SysCtlPeripheralEnable(M2_IN_PERIPH);
    GPIOPinTypeGPIOOutput(M2_PORT, M2_IN1_PIN | M2_IN2_PIN);
    GPIOPinWrite(M2_PORT, M2_IN1_PIN | M2_IN2_PIN, 0);

    //* Pwm signals
    SysCtlPeripheralEnable(M2_PWM_PERIPH);
    GPIOPinConfigure(M2_PWM_PINMUX); // m2
    GPIOPinTypePWM(M2_PORT, M2_PWM_PIN);
    PWMGenConfigure(M2_PWM_BASE, M2_PWM_GEN, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    //* period
    PWMGenPeriodSet(M2_PWM_BASE, M2_PWM_GEN, M_PWM_PERIOD);
    PWMPulseWidthSet(M2_PWM_BASE, M2_PWM_OUT, 1);

    PWMGenEnable(M2_PWM_BASE, M2_PWM_GEN);

    PWMOutputState(M2_PWM_BASE, M2_PWM_OUT_BIT, true);
}

void InitEncoder1()
{
    //* enable QEI periph
    SysCtlPeripheralEnable(ENC1_PERIPH);
    SysCtlPeripheralEnable(ENC1_PINS_PERIPH);
    while(!SysCtlPeripheralReady(ENC1_PERIPH))
    {
    }
    // // Unlock (only for PD7)
    // HWREG(M2_ENC_GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
	// HWREG(M2_ENC_GPIO_BASE + GPIO_O_CR) |= 0x80;
	// HWREG(M2_ENC_GPIO_BASE + GPIO_O_AFSEL) &= ~0x80;
    //* configure phase pins
    GPIOPinConfigure(ENC1_A_PINMUX);
    GPIOPinConfigure(ENC1_B_PINMUX);

    //* set GPIO Pins
    GPIOPinTypeQEI(ENC1_GPIO_BASE, ENC1_A_PIN | ENC1_B_PIN);
    
    // disable befor configuration
    // QEIDisable(ENC1_BASE);
    // QEIIntDisable(ENC1_BASE, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    //* configure
    QEIConfigure(ENC1_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE |QEI_CONFIG_SWAP, 0xffffffff);
    QEIVelocityConfigure(ENC1_BASE, QEI_VELDIV_1, LOOP_TIMER_PERIOD); //

    //* enable
    QEIEnable(ENC1_BASE);
    QEIVelocityEnable(ENC1_BASE);
    //* set position 
    QEIPositionSet(ENC1_BASE, 0);
    
    // motor(0);

}

void InitEncoder2()
{
    //* enable QEI periph
    SysCtlPeripheralEnable(ENC2_PERIPH);
    SysCtlPeripheralEnable(ENC2_PINS_PERIPH);
    while(!SysCtlPeripheralReady(ENC2_PERIPH))
    {
    }
    // // Unlock (only for PD7)
    HWREG(ENC2_GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
	HWREG(ENC2_GPIO_BASE + GPIO_O_CR) |= 0x80;
	HWREG(ENC2_GPIO_BASE + GPIO_O_AFSEL) &= ~0x80;
    //* configure phase pins
    GPIOPinConfigure(ENC2_A_PINMUX);
    GPIOPinConfigure(ENC2_B_PINMUX);

    //* set GPIO Pins
    GPIOPinTypeQEI(ENC2_GPIO_BASE, ENC2_A_PIN | ENC2_B_PIN);
    
    // disable befor configuration
    // QEIDisable(ENC2_BASE);
    // QEIIntDisable(ENC2_BASE, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    //* configure
    QEIConfigure(ENC2_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE |QEI_CONFIG_NO_SWAP, 0xffffffff);
    QEIVelocityConfigure(ENC2_BASE, QEI_VELDIV_1, LOOP_TIMER_PERIOD); //

    //* enable
    QEIEnable(ENC2_BASE);
    QEIVelocityEnable(ENC2_BASE);
    //* set position 
    QEIPositionSet(ENC2_BASE, 0);
    
    // motor(0);

}

void InitTimer0()
{
    //* enable clock for timer 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    //* configure as periodic
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //* load set value
    TimerLoadSet(TIMER0_BASE, TIMER_A, LOOP_TIMER_PERIOD);
    //* enable timer interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //* set the isr
    TimerIntRegister(TIMER0_BASE, TIMER_A, loopISR);
    //* enable interrupt in NVIC
    IntEnable(INT_TIMER0A);
    //* enable timer
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void InitSystick()
{
    SysTickPeriodSet(SYSTICK_PERIOD);
    SysTickIntRegister(SystickCB);
    SysTickIntEnable();
    SysTickEnable();
}

void InitConsole()
{
    //* enable gpio periph
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //* configure pinmux
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    //* enable uart
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    //* use internal 16MHz oscilator
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    //* select alternate functions for pins
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //* init uart for console I/O
    UARTStdioConfig(0, 115200, 16000000);


}

void InitTemp()
{
    //* enable peroph
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    //* enable  samble seq with processor trigger
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    //* configure step 0 on seq 3, sample temp sensor and interrupt when is done.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);
    //* enable adc seq
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}
//
void setLED(uint32_t red, uint32_t green)
{
    PWMPulseWidthSet(LED_PWM_BASE, RED_LED_OUT, red);
    PWMPulseWidthSet(LED_PWM_BASE, GREEN_LED_OUT, green);
    
}
//
void lmotor(int vel)
{
    if (vel > 0)
        GPIOPinWrite(M1_PORT, M1_IN1_PIN | M1_IN2_PIN, M1_IN1_PIN);
    else
        GPIOPinWrite(M1_PORT, M1_IN1_PIN | M1_IN2_PIN, M1_IN2_PIN);
    uint32_t fin_value = abs(vel);
    if (fin_value == 0)
        GPIOPinWrite(M1_PORT, M1_IN1_PIN | M1_IN2_PIN, 0);
    PWMPulseWidthSet(M1_PWM_BASE, M1_PWM_OUT, fin_value);
}

void rmotor(int vel)
{
    if (vel > 0)
        GPIOPinWrite(M2_PORT, M2_IN1_PIN | M2_IN2_PIN, M2_IN1_PIN);
    else
        GPIOPinWrite(M2_PORT, M2_IN1_PIN | M2_IN2_PIN, M2_IN2_PIN);
    uint32_t fin_value = abs(vel);
    if (fin_value == 0)
        GPIOPinWrite(M2_PORT, M2_IN1_PIN | M2_IN2_PIN, 0);
    PWMPulseWidthSet(M2_PWM_BASE, M2_PWM_OUT, fin_value);
}

// enc 
uint32_t getLMotorTicks()
{
    return QEIPositionGet(ENC1_BASE);
}
int32_t getLMotorVel()
{
    return QEIDirectionGet(ENC1_BASE) * QEIVelocityGet(ENC1_BASE);
}
uint32_t getRMotorTicks()
{
    return QEIPositionGet(ENC2_BASE);
}
int32_t getRMotorVel()
{
    return QEIDirectionGet(ENC2_BASE) * QEIVelocityGet(ENC2_BASE);
}

uint32_t readTempC()
{
    uint32_t adc_value;
    //* trigger conversion
    ADCProcessorTrigger(ADC0_BASE, 3);
    //* wait for conversion to be completed
    while(!ADCIntStatus(ADC0_BASE, 3, false)){}
    //* clear flag
    ADCIntClear(ADC0_BASE, 3);
    //* read adc value
    ADCSequenceDataGet(ADC0_BASE, 3, &adc_value);
    //* convert according to datasheet
    uint32_t temp_C = ((1475 * 1023) - (2250 * adc_value)) / 10230;

    return temp_C;
}

//* ISR
void loopISR()
{
    //* clear interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    c_flag = true;
}

//* Serial commands
void serialCommand(char *buffer)
{
    switch (buffer[0])
    {
    case 'r':
        // UARTprintf('r\n');
        break;
    case 'w':
        // UARTprintf('w\n');
        break;
    
    default:
        break;
    }
}
