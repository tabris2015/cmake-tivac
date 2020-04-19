//*****************************************************************************
//
// blinky.cpp - Simple example to blink the on-board LED.
//
//*****************************************************************************
// #define PART_TM4C1230C3PM
#include <stdbool.h>
#include <stdint.h>
#include <cstdlib> // abs function
#include <inc/tm4c123gh6pm.h>
#include <inc/hw_gpio.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>

#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/gpio.h>
#include <driverlib/pwm.h>
#include <driverlib/qei.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>


// #include "pinout.h"

// rgb led defines
#define LED_PORT GPIO_PORTF_BASE
#define LED_PWM_BASE PWM1_BASE
#define LED_PWM_PERIPH SYSCTL_PERIPH_PWM1
#define LED_PERIPH SYSCTL_PERIPH_GPIOF

#define RED_LED_PINMUX GPIO_PF1_M1PWM5
#define RED_LED_PIN GPIO_PIN_1
#define RED_LED_GEN PWM_GEN_2
#define RED_LED_OUT PWM_OUT_5
#define RED_LED_OUT_BIT PWM_OUT_5_BIT

#define GREEN_LED_PINMUX GPIO_PF3_M1PWM7
#define GREEN_LED_PIN GPIO_PIN_3
#define GREEN_LED_GEN PWM_GEN_3
#define GREEN_LED_OUT PWM_OUT_7
#define GREEN_LED_OUT_BIT PWM_OUT_7_BIT
// #define RED_LED_PORT GPIO_PORTF_BASE

#define M_PWM_PERIOD 999

// M1 defines
#define M1_PORT GPIO_PORTD_BASE
// PWM
#define M1_PWM_BASE PWM1_BASE
#define M1_PWM_PERIPH SYSCTL_PERIPH_PWM1
#define M1_PWM_PINMUX GPIO_PD0_M1PWM0
#define M1_PWM_PIN GPIO_PIN_0
#define M1_PWM_GEN PWM_GEN_0
#define M1_PWM_OUT PWM_OUT_0
#define M1_PWM_OUT_BIT PWM_OUT_0_BIT
// GPIO
#define M1_IN_PERIPH SYSCTL_PERIPH_GPIOD
#define M1_IN1_PIN GPIO_PIN_1
#define M1_IN2_PIN GPIO_PIN_2

// M2 defines
#define M2_PORT GPIO_PORTB_BASE
// PWM
#define M2_PWM_BASE PWM0_BASE
#define M2_PWM_PERIPH SYSCTL_PERIPH_PWM0
#define M2_PWM_PINMUX GPIO_PB4_M0PWM2
#define M2_PWM_PIN GPIO_PIN_4
#define M2_PWM_GEN PWM_GEN_1
#define M2_PWM_OUT PWM_OUT_2
#define M2_PWM_OUT_BIT PWM_OUT_2_BIT
// GPIO
#define M2_IN_PERIPH SYSCTL_PERIPH_GPIOB
#define M2_IN1_PIN GPIO_PIN_0
#define M2_IN2_PIN GPIO_PIN_1

// Encoders defines
#define ENC_PPR (380*4*7)
// encoder 1
#define ENC1_PERIPH SYSCTL_PERIPH_QEI1
#define ENC1_PINS_PERIPH SYSCTL_PERIPH_GPIOC
#define ENC1_BASE QEI1_BASE
#define ENC1_GPIO_BASE GPIO_PORTC_BASE
#define ENC1_A_PINMUX GPIO_PC5_PHA1
#define ENC1_B_PINMUX GPIO_PC6_PHB1
#define ENC1_A_PIN GPIO_PIN_5
#define ENC1_B_PIN GPIO_PIN_6

// encoder 2
#define ENC2_PERIPH SYSCTL_PERIPH_QEI0
#define ENC2_PINS_PERIPH SYSCTL_PERIPH_GPIOD
#define ENC2_BASE QEI0_BASE
#define ENC2_GPIO_BASE GPIO_PORTD_BASE
#define ENC2_A_PINMUX GPIO_PD6_PHA0
#define ENC2_B_PINMUX GPIO_PD7_PHB0
#define ENC2_A_PIN GPIO_PIN_6
#define ENC2_B_PIN GPIO_PIN_7

// #include "pin.h"

// globals
volatile int qei_pos;

volatile uint32_t led_pwm = 0;
volatile uint32_t red_pwm = 0;
volatile uint32_t green_pwm = 0;

volatile uint8_t state = 0;
// #define delay(x)      SysCtlDelay(SysCtlClockGet() / 3 * x);

void delayMS(int ms)
{
    SysCtlDelay((SysCtlClockGet() / (3 * 1000)) * ms);
}
// prototypes
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

// prototypes for motors api
void lmotor(int vel);
void rmotor(int vel);

// prototypes for encoder

// led api
void setLED(uint32_t red, uint32_t green);

// timer callback
void timeout();

//*****************************************************************************
// Blink the on-board LED.
//*****************************************************************************
int main(void)
{
    InitBoard();
    delayMS(500);
    setLED(1, 1);
    delayMS(500);
    
    while (1)
    {
        // red_pwm = (QEIPositionGet(ENC1_BASE) * M_PWM_PERIOD) / ENC_PPR;
        // green_pwm = (QEIPositionGet(ENC2_BASE) * M_PWM_PERIOD) / ENC_PPR;
        // setLED(red_pwm, green_pwm);
        // lmotor(red_pwm);
        // rmotor(red_pwm);
        delayMS(5);
    }
}

// functions
void InitBoard()
{
    InitCLock();
    InitLED();
    InitEncoder1();
    InitEncoder2();
    InitLeftMotor();
    InitRightMotor();
    InitTimer0();
    IntMasterEnable();
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
    // Digital pins for direction
    SysCtlPeripheralEnable(M1_IN_PERIPH);
    GPIOPinTypeGPIOOutput(M1_PORT, M1_IN1_PIN | M1_IN2_PIN);
    GPIOPinWrite(M1_PORT, M1_IN1_PIN | M1_IN2_PIN, 0);

    // Pwm signals
    SysCtlPeripheralEnable(M1_PWM_PERIPH);
    GPIOPinConfigure(M1_PWM_PINMUX); // m1
    GPIOPinTypePWM(M1_PORT, M1_PWM_PIN);
    PWMGenConfigure(M1_PWM_BASE, M1_PWM_GEN, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    // period
    PWMGenPeriodSet(M1_PWM_BASE, M1_PWM_GEN, M_PWM_PERIOD);
    PWMPulseWidthSet(M1_PWM_BASE, M1_PWM_OUT, 1);

    PWMGenEnable(M1_PWM_BASE, M1_PWM_GEN);

    PWMOutputState(M1_PWM_BASE, M1_PWM_OUT_BIT, true);
}

void InitRightMotor()
{
    // Digital pins for direction
    SysCtlPeripheralEnable(M2_IN_PERIPH);
    GPIOPinTypeGPIOOutput(M2_PORT, M2_IN1_PIN | M2_IN2_PIN);
    GPIOPinWrite(M2_PORT, M2_IN1_PIN | M2_IN2_PIN, 0);

    // Pwm signals
    SysCtlPeripheralEnable(M2_PWM_PERIPH);
    GPIOPinConfigure(M2_PWM_PINMUX); // m2
    GPIOPinTypePWM(M2_PORT, M2_PWM_PIN);
    PWMGenConfigure(M2_PWM_BASE, M2_PWM_GEN, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    // period
    PWMGenPeriodSet(M2_PWM_BASE, M2_PWM_GEN, M_PWM_PERIOD);
    PWMPulseWidthSet(M2_PWM_BASE, M2_PWM_OUT, 1);

    PWMGenEnable(M2_PWM_BASE, M2_PWM_GEN);

    PWMOutputState(M2_PWM_BASE, M2_PWM_OUT_BIT, true);
}

void InitEncoder1()
{
    // enable QEI periph
    SysCtlPeripheralEnable(ENC1_PERIPH);
    SysCtlPeripheralEnable(ENC1_PINS_PERIPH);
    while(!SysCtlPeripheralReady(ENC1_PERIPH))
    {
    }
    // // Unlock (only for PD7)
    // HWREG(M2_ENC_GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
	// HWREG(M2_ENC_GPIO_BASE + GPIO_O_CR) |= 0x80;
	// HWREG(M2_ENC_GPIO_BASE + GPIO_O_AFSEL) &= ~0x80;
    // configure phase pins
    GPIOPinConfigure(ENC1_A_PINMUX);
    GPIOPinConfigure(ENC1_B_PINMUX);

    // set GPIO Pins
    GPIOPinTypeQEI(ENC1_GPIO_BASE, ENC1_A_PIN | ENC1_B_PIN);
    
    // disable befor configuration
    // QEIDisable(ENC1_BASE);
    // QEIIntDisable(ENC1_BASE, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    // configure
    QEIConfigure(ENC1_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE |QEI_CONFIG_NO_SWAP, ENC_PPR);

    // enable
    QEIEnable(ENC1_BASE);

    // set position 
    QEIPositionSet(ENC1_BASE, 1);
    
    // motor(0);

}

void InitEncoder2()
{
    // enable QEI periph
    SysCtlPeripheralEnable(ENC2_PERIPH);
    SysCtlPeripheralEnable(ENC2_PINS_PERIPH);
    while(!SysCtlPeripheralReady(ENC2_PERIPH))
    {
    }
    // // Unlock (only for PD7)
    HWREG(ENC2_GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
	HWREG(ENC2_GPIO_BASE + GPIO_O_CR) |= 0x80;
	HWREG(ENC2_GPIO_BASE + GPIO_O_AFSEL) &= ~0x80;
    // configure phase pins
    GPIOPinConfigure(ENC2_A_PINMUX);
    GPIOPinConfigure(ENC2_B_PINMUX);

    // set GPIO Pins
    GPIOPinTypeQEI(ENC2_GPIO_BASE, ENC2_A_PIN | ENC2_B_PIN);
    
    // disable befor configuration
    // QEIDisable(ENC2_BASE);
    // QEIIntDisable(ENC2_BASE, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    // configure
    QEIConfigure(ENC2_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE |QEI_CONFIG_NO_SWAP, ENC_PPR);

    // enable
    QEIEnable(ENC2_BASE);

    // set position 
    QEIPositionSet(ENC2_BASE, 1);
    
    // motor(0);

}

void InitTimer0()
{
    // enable clock for timer 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // configure as periodic
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    // load set value
    TimerLoadSet(TIMER0_BASE, TIMER_A, 20000000);
    // enable timer interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // set the isr
    TimerIntRegister(TIMER0_BASE, TIMER_A, timeout);
    // enable interrupt in NVIC
    IntEnable(INT_TIMER0A);
    // enable timer
    TimerEnable(TIMER0_BASE, TIMER_A);
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

void timeout()
{
    // clear interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    if(state)
        setLED(0,0);
    else
        setLED(0,500);

    state = !state;
}