/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "TimerA1.h"
#include "timer_a.h"

// Pointer to user function that gets called on timer interrupt
#define TIMER_INTERRUPT_PRIORITY 0
void (*TimerA1Task)(void);

// Initializes Timer A1 in up mode and triggers a periodic interrupt at a desired frequency
// Timer is running at 500kHz frequency, the period indicates the number of cycles at this frequency for there to be an interrupt
void TimerA1_Init(void(*task)(void), uint16_t period){
    TimerA1Task = task;

    // C struct to pass into the timer A1 register, this is processed and used by timer_a and
    Timer_A_UpModeConfig Config =
    {
     // Use the 48MhZ
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_24,
     500,
     TIMER_A_TAIE_INTERRUPT_DISABLE,
     TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
     TIMER_A_DO_CLEAR
    };

    // Initializes Timer A1 in up mode
    Config.timerPeriod = period;
    Timer_A_configureUpMode(TIMER_A1_BASE, &Config);
    // configure interrupts on timer
    Interrupt_enableInterrupt(INT_TA1_0);
    Interrupt_enableMaster();
    // Starts the TimerA1
    Timer_A_clearTimer(TIMER_A1_BASE);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}

// stops Timer A1
void TimerA1_Stop(void){
    // Looks into the Timer_A1 control register, masks it to set the 4th and 5th bit to zero, and then sets the register to the masked value
    // The 4th and 5th bits are the active bits for the timer, By setting those bits in the register to zero, the Timer stops
    TIMER_A1->CTL = TIMER_A1->CTL & 0xFFCF;
}

// ISR function for Timer A1 periodic interrupt
void TA1_0_IRQHandler(void){
    // Call the task
    TimerA1Task();

    //Clear Interrupt tasks
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}
