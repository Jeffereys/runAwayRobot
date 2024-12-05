// Motor Control Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// Servo motor drive:
//   PWM1 output on M1PWM2 (PA6)
//   DIR1 output on PA7
//   PWM3 output on M1PWM6 (PF2) - blue on-board LED
//   DIR3 output on PF3 - green on-board LED

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "motor_control.h"
#include "gpio.h"

// Bitband aliases
#define DIRECTION1    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define DIRECTION3    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// Pins
#define PWM1 PORTA,6
#define DIR1 PORTA,7
#define PWM3 PORTF,2
#define DIR3 PORTF,3

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize motor control
void initMotorControl()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    _delay_cycles(3);
    enablePort(PORTA);
    enablePort(PORTF);

    // Configure PWM and DIRECTION outputs
    selectPinPushPullOutput(DIR1);
    selectPinPushPullOutput(PWM1);
    setPinAuxFunction(PWM1, GPIO_PCTL_PA6_M1PWM2);

    selectPinPushPullOutput(DIR3);
    selectPinPushPullOutput(PWM3);
    setPinAuxFunction(PWM3, GPIO_PCTL_PF2_M1PWM6);

    // Configure PWM module 1 to drive servo
    // PWM on M1PWM6 (PF2), M0PWM3a
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R1;                          // reset PWM1 module
    SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R1;                         // leave reset state
    _delay_cycles(3);                                           // wait 3 clocks

    PWM1_1_CTL_R = 0;                                           // turn-off PWM1 generator 1
    PWM1_3_CTL_R = 0;                                           // turn-off PWM1 generator 3

    PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;

    PWM1_1_LOAD_R = 1024;                                       // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 1024;                                       // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    
    PWM1_1_CMPA_R = 0;                                          // PWM off (0=always low, 1023=always high)
    PWM1_3_CMPA_R = 0;                                          // PWM off (0=always low, 1023=always high)

    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;                            // turn-on PWM1 generator 1
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                            // turn-on PWM1 generator 3

    PWM1_ENABLE_R = PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM6EN;      // enable PWM output
}

void setMotorPwm1(unsigned int dutyCycle)
{
    PWM1_1_CMPA_R = dutyCycle;
}

void setMotorDirection1(bool dir)
{
    DIRECTION1 = dir;
}

void setMotorPwm3(unsigned int dutyCycle)
{
    PWM1_3_CMPA_R = dutyCycle;
}

void setMotorDirection3(bool dir)
{
    DIRECTION3 = dir;
}
