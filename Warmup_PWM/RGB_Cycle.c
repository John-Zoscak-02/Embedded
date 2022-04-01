/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>

/* Project Includes */
#include "RGB_PWM.h"
#include "RGB_Cycle.h"

/* Application Defines  */
#define TIMERA0_PERIOD       0xFFFF // 65535
#define BLUE_CCR_VALUE      (TIMERA0_PERIOD/4)
#define GREEN_CCR_VALUE     (TIMERA0_PERIOD/2)
#define BLUE_CCR_VALUE2   3*(TIMERA0_PERIOD/4)
#define RED_CCR_VALUE       (TIMERA0_PERIOD)


/* Timer_A Continuous Configuration Parameter */
const Timer_A_ContinuousModeConfig continuousModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,           // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,      // ACLK/1 = 32.768khz
        TIMER_A_TAIE_INTERRUPT_DISABLE,     // Enable Overflow ISR
        TIMER_A_DO_CLEAR                    // Clear Counter
};

uint16_t Current_Duty;

// Initializes TimerA1 to output a cycle of the RGB color combinations
void Spectrum_Init(uint16_t duty) {

    // Assign the Default Duty Cycle for the RGB LEDs according to the argument
    Current_Duty = duty;

    // Configuring Continuous Mode
    MAP_Timer_A_configureContinuousMode(TIMER_A0_BASE, &continuousModeConfig);

    // Configure Capture Compare Interrupts on CCR1-CCR4
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, BLUE_CCR_VALUE);
    Timer_A_enableCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, GREEN_CCR_VALUE);
    Timer_A_enableCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);

    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, BLUE_CCR_VALUE2);
    Timer_A_enableCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3);

    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, RED_CCR_VALUE);
    Timer_A_enableCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4);

    // Enabling interrupts
    Interrupt_enableInterrupt(INT_TA0_N);

    // Starting the Timer_A0 in continuous mode
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
}


// Sets the duty cycle of the RGB LEDs in the Spectrum
void Set_Spectrum_Duty(uint16_t duty) {
    Current_Duty = duty;
}



//******************************************************************************
//
//This is the TIMERA0 interrupt vector service routine.
//
//******************************************************************************
void TA0_N_IRQHandler(void) {

    // Check if the CCR1 Flag was triggered
    if (Timer_A_getCaptureCompareInterruptStatus(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG) == 1) {

        Toggle_Red_LED(Current_Duty);

        // Clear the CCR1 Interrupt Flag
        Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_1);
    }

    // Check if the CCR2 Flag was triggered
    else if (Timer_A_getCaptureCompareInterruptStatus(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG) == 1) {

        Toggle_Red_LED(Current_Duty);
        Toggle_Green_LED(Current_Duty);

        // Clear the CCR2 Interrupt Flag
        Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_2);
    }

    // Check if the CCR3 Flag was triggered
    else if (Timer_A_getCaptureCompareInterruptStatus(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG) == 1) {

        Toggle_Red_LED(Current_Duty);

        // Clear the CCR3 Interrupt Flag
        Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_3);
    }

    // Check if the CCR4 Flag was triggered
    else if (Timer_A_getCaptureCompareInterruptStatus(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG) == 1) {

        Toggle_Red_LED(Current_Duty);
        Toggle_Green_LED(Current_Duty);
        Toggle_Blue_LED(Current_Duty);

        // Clear the CCR4 Interrupt Flag
        Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_4);
    }
}
