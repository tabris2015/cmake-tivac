//*****************************************************************************
//
// blinky.cpp - Simple example to blink the on-board LED.
//
//*****************************************************************************
#define PART_TM4C1230C3PM
#include <stdbool.h>
#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include <inc/hw_gpio.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>

#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/pwm.h>
#include <driverlib/pin_map.h>

// #include "pin.h"

// #define delay(x)      SysCtlDelay(SysCtlClockGet() / 3 * x);

void delayMS(int ms)
{
    SysCtlDelay((SysCtlClockGet() / (3 * 1000)) * ms);
}

//*****************************************************************************
// Blink the on-board LED.
//*****************************************************************************
int main(void)
{
    // // Clock (80MHz)
    // SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 320);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 100);
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);

    //Fade
    bool fadeUp = true;
    unsigned long increment = 10;
    unsigned long pwmNow = 100;

    while (1)
    {
        delayMS(20);
        if (fadeUp)
        {
            pwmNow += increment;
            if (pwmNow >= 320)
            {
                fadeUp = false;
            }
        }
        else
        {
            pwmNow -= increment;
            if (pwmNow <= 10)
            {
                fadeUp = true;
            }
        }
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, pwmNow);
        // PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, pwmNow);
        // PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, pwmNow);
    }
    // // Setup Red Led
    // Pin led = Pin(PF2);
    // led.setDir(OUT);

    // while(1){
    //   led.toggle();
    //   delayMS(50);
    // }
}
