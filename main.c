//Jefferey Stephens
//Khoi Tran
//David Duong

///////-----------------------------------------Run Away! Robot-----------------------------------------------------///
/*
 * The Run Away! Robot uses 4 ultrasonic sensors to detect an obstacle that is withing a given range 0-200 mm
 * The robot will interpret the distance and send according motor controls to avoid the obstacle.
 *
 * A basic function creating a star pattern will be used to demonstrate the effect.
 *
 * During patterend movement, the robot will follow a set path. As the robot approcahes an obstacle it will turn and speed
 * away for a set distance. After which, the patteren will resume.
 *
 */
///////---------------------------------------------------------------------------------------------------------------///
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "motor_control.h"
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

// Define TRIG and ECHO pins
// TRIG_1   PB7
#define TRIG_1       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 7*4)))
//ECHO_1    PC6
//TRIG_2    PB6
#define TRIG_2       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
//ECHO_2    PC7
//TRIG_3    PA4
#define TRIG_3       (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
//ECHO_3    PD6
// TRIG_4 PE2
#define TRIG_4       (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))
// ECHO_4 PC4

// TRIG and ECHO masks
#define TRIG_1_MASK  128
#define ECHO_1_MASK  64
#define TRIG_2_MASK  64
#define ECHO_2_MASK  128
#define TRIG_3_MASK  16
#define ECHO_3_MASK  64
#define TRIG_4_MASK  4
#define ECHO_4_MASK  16

uint32_t time = 0;
uint32_t frequency = 0;
uint32_t CURRENT_PWM        = 95; //0-100%

int forward = 1;
int reverse = 0;

int threshold = 400; // distance where the sensors pick up
double star = 8;    //relative length of star formation

void enableTimerMode()
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER1_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R = 1 << (INT_WTIMER1A-16-96);          // turn-on interrupt 112 (WTIMER1A)
}

void disableTimerMode()
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter
    NVIC_DIS3_R = 1 << (INT_WTIMER1A-16-96);         // turn-off interrupt 112 (WTIMER1A)
}

// Period timer service publishing latest time measurements every positive edge
void wideTimer1Isr()
{
    time = WTIMER1_TAV_R;                        // read counter input
    WTIMER1_TAV_R = 0;                           // zero counter for next edge
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

// Frequency counter service publishing latest frequency measurements every second
void timer1Isr()
{
    frequency = WTIMER1_TAV_R;                   // read counter input
    WTIMER1_TAV_R = 0;                           // reset counter for next period
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;           // clear interrupt flag
}

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0 | SYSCTL_RCGCWTIMER_R1 |SYSCTL_RCGCWTIMER_R2|SYSCTL_RCGCWTIMER_R3|SYSCTL_RCGCWTIMER_R4| SYSCTL_RCGCWTIMER_R5;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure TRIG and ECHO pins
    GPIO_PORTB_DIR_R |= TRIG_1_MASK | TRIG_2_MASK;  // Set PB7 and PB6 are outputs
    GPIO_PORTB_DR2R_R |= TRIG_1_MASK | TRIG_2_MASK; // Set drive strength to 2mA
    GPIO_PORTB_DEN_R |= TRIG_1_MASK | TRIG_2_MASK;  // Enable TRIG

    GPIO_PORTA_DIR_R |= TRIG_3_MASK;    // Set PA4 is output
    GPIO_PORTA_DR2R_R |= TRIG_3_MASK;   // Set drive strength to 2mA
    GPIO_PORTA_DEN_R |= TRIG_3_MASK;    // Enable TRIG

    GPIO_PORTE_DIR_R |= TRIG_4_MASK;
    GPIO_PORTE_DR2R_R |= TRIG_4_MASK;
    GPIO_PORTE_DEN_R |= TRIG_4_MASK;

    GPIO_PORTC_DIR_R &= ~ECHO_1_MASK | ~ECHO_2_MASK | ~ECHO_4_MASK;                    // Set PC6 and PC7 are outputs
    GPIO_PORTC_DR2R_R |= ECHO_1_MASK | ECHO_2_MASK | ECHO_4_MASK;                     // Set drive strength to 2mA
    GPIO_PORTC_AFSEL_R |= ECHO_1_MASK | ECHO_2_MASK | ECHO_4_MASK;                    // select alt fns for PC6 and PC7
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M | ~GPIO_PCTL_PC7_M | ~GPIO_PCTL_PC4_M;           // map alt fns for PC6 and PC7
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0 | GPIO_PCTL_PC7_WT1CCP1 | GPIO_PCTL_PC4_WT0CCP0;
    GPIO_PORTC_DEN_R |= ECHO_1_MASK | ECHO_2_MASK | ECHO_4_MASK;                      // enable PC6 and PC7 for digital input

    GPIO_PORTD_DIR_R &= ~ECHO_3_MASK;            // Set PD7 is input
    GPIO_PORTD_DR2R_R |= ECHO_3_MASK;            // Set drive strength to 2mA
    GPIO_PORTD_AFSEL_R |= ECHO_3_MASK;           // select alt fns for PD6
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PD6_M;       // map alt fns for PD6
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PD6_WT5CCP0;
    GPIO_PORTD_DEN_R |= ECHO_3_MASK;             // enable PD7 for digital input
}

uint32_t measure_mm(uint8_t n)
{
    uint32_t distance = 0;
    uint32_t microTime = 0;
    uint32_t delayTime = 10000;

    switch(n)
    {
        case 0:
            // Enable the TRIG at least 10 us
            TRIG_1 = 1;
            waitMicrosecond(10);
            TRIG_1 = 0;

            // // Start timer
            enableTimerMode();

            // Wait for the rising edge and count down delay time
            while(!(GPIO_PORTC_DATA_R & ECHO_1_MASK) && delayTime)
            {
                delayTime--;
            }

            // If delay time is equal to 0, stop timer and return distance value 0
            if(delayTime == 0)
            {
                disableTimerMode();
                return 0;
            }

            WTIMER1_TAV_R = 0;                      // Reset timer
            while(GPIO_PORTC_DATA_R & ECHO_1_MASK); // Wait for the falling edge
            microTime = WTIMER1_TAV_R / 40;         // Calculate time in microseconds
            disableTimerMode();                     // Stop timer

            // Convert distance in mm
            distance = microTime * 0.345 / 2;

            return distance;

        case 1:
            // Enable the TRIG at least 10 us
            TRIG_2 = 1;
            waitMicrosecond(10);
            TRIG_2 = 0;

            // Start timer
            enableTimerMode();

            // Wait for the rising edge and count down delay time
            while(!(GPIO_PORTC_DATA_R & ECHO_2_MASK) && delayTime)
            {
                delayTime--;
            }

            // If delay time is equal to 0, stop timer and return distance value 0
            if(delayTime == 0)
            {
                disableTimerMode();
                return 0;
            }

            WTIMER1_TAV_R = 0;                      // Reset timer
            while(GPIO_PORTC_DATA_R & ECHO_2_MASK); // Wait for the falling edge
            microTime = WTIMER1_TAV_R / 40;         // Calculate time in microseconds
            disableTimerMode();                     // Stop timer

            // Convert distance in mm
            distance = microTime * 0.345 / 2;

            return distance;

        case 2:
            // Enable the TRIG at least 10 us
            TRIG_3 = 1;
            waitMicrosecond(10);
            TRIG_3 = 0;

            // Start timer
            enableTimerMode();

            // Wait for the rising edge and count down delay time
            while(!(GPIO_PORTD_DATA_R & ECHO_3_MASK) && delayTime)
            {
                delayTime--;
            }

            // If delay time is equal to 0, stop timer and return distance value 0
            if(delayTime == 0)
            {
                disableTimerMode();
                return 0;
            }

            WTIMER1_TAV_R = 0;                      // Reset timer
            while(GPIO_PORTD_DATA_R & ECHO_3_MASK); // Wait for the falling edge
            microTime = WTIMER1_TAV_R / 40;         // Calculate time in microseconds
            disableTimerMode();                     // Stop timer

            // Convert distance in mm
            distance = microTime * 0.345 / 2;

            return distance;

        case 3:
            // Enable the TRIG at least 10 us
            TRIG_4 = 1;
            waitMicrosecond(10);
            TRIG_4 = 0;

            // Start timer
            enableTimerMode();

            // Wait for the rising edge and count down delay time
            while(!(GPIO_PORTC_DATA_R & ECHO_4_MASK) && delayTime)
            {
                delayTime--;
            }

            // If delay time is equal to 0, stop timer and return distance value 0
            if(delayTime == 0)
            {
                disableTimerMode();
                return 0;
            }

            WTIMER1_TAV_R = 0;                      // Reset timer
            while(GPIO_PORTC_DATA_R & ECHO_4_MASK); // Wait for the falling edge
            microTime = WTIMER1_TAV_R / 40;         // Calculate time in microseconds
            disableTimerMode();                     // Stop timer

            // Convert distance in mm
            distance = microTime * 0.345 / 2;

            return distance;

            default:
                return 0;
        }
}

void motorRControl(unsigned int speed, bool direction)
{
    setMotorPwm1(speed);
    setMotorDirection1(direction);
}

void motorLControl(unsigned int speed, bool direction)
{
    setMotorPwm3(speed);
    setMotorDirection3(direction);
}
bool starFormation(uint32_t foo)
{
    if (foo < star)
    {
//        motorRControl(900, 0);
//        motorLControl(900, 0);
        motorRControl(0, 0);
                motorLControl(0, 0);
    }
    else if (foo < star*2)
    {
//        motorRControl(799, 0);
//        motorLControl(999, 0);
        motorRControl(0, 0);
                motorLControl(0, 0);
    }
    else
        return 0;

    return 1;
}


void setMotorControl(uint32_t distLeft, uint32_t distFront, uint32_t distRight, uint32_t distBack , uint32_t foo )
{


    if(distLeft < threshold)
        {
            motorRControl(815, 0);
            motorLControl(1023, 0);
        }
    else if(distFront < threshold)
        {
            motorRControl(1023, 0);
            motorLControl(1, 1);
        }
    else if(distRight < threshold)
        {
            motorRControl(1023, 0);
            motorLControl(815, 0);
        }
    else if(distBack < threshold && distBack != 0)
        {
             motorRControl(1023, 0);
             motorLControl(1023, 0);
        }

        else
        {
            motorRControl(710, 0);
                    motorLControl(710, 0);
//            motorRControl(0, 0);
//                    motorLControl(0, 0);

           // starFormation(foo);     //PROD

        }

}

int main (void)
{
    initHw();
    initUart0();
    initMotorControl();

    uint32_t distLeft = 0, distFront = 0, distRight = 0, distBack = 0;

    setUart0BaudRate(115200, 40e6);
//    char str0[40], str1[40], str2[40], str3[40];
    char str0[40];


int foo = 0;

            while(true)
                {
                    distLeft  = measure_mm(0);

                    distFront = measure_mm(1);

                    distRight = measure_mm(2);

                    distBack = measure_mm(3);

                    snprintf(str0, sizeof(str0), "Distance LEFT     || %d mm\n", distLeft);
                    putsUart0(str0);
                    snprintf(str0, sizeof(str0), "Distance FRONT    || %d mm\n", distFront);
                    putsUart0(str0);
                    snprintf(str0, sizeof(str0), "Distance RIGHT    || %d mm\n", distRight);
                    putsUart0(str0);
                    snprintf(str0, sizeof(str0), "Distance BACK     || %d mm\n", distBack);
                    putsUart0(str0);
                    putsUart0("\n");

                    setMotorControl( distLeft,  distFront,  distRight,  distBack, foo);

                    waitMicrosecond(1000);
//                    if(!starFormation(foo))
//                    {
//                        foo = 0;
//                    }
//                    foo++;
                }

    return 0;
}
