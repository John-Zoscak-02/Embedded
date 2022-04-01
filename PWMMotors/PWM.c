/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>

/* Project Includes */
#include "PWM.h"

/* Application Defines  */
#define TIMERA0_PERIOD 127

/* Timer_A UpDown Configuration Parameter */
Timer_A_UpDownModeConfig upDownConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,               // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 12mHz
        TIMERA0_PERIOD,                         // 127 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

Timer_A_CompareModeConfig compareConfig_PWM3 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_3,          // Use CCR3
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_SET,              // Toggle output bit
        TIMERA0_PERIOD                              // Initial 0% Duty Cycle
};

/* Timer_A Compare Configuration Parameter (PWM2) */
Timer_A_CompareModeConfig compareConfig_PWM4 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_4,          // Use CCR4
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_SET,              // Toggle output bit
        TIMERA0_PERIOD                              // Initial 0% Duty Cycle
};

/**
 * Initializes PWM output on P2.6 (right motor) using Timer A0 CCR3 interrupt
 * and on P2.7 (left motor) using the Timer A0 CCR4 interrupt
 * Assumes SMCLK has been initialized to 12 MHz
 * @param period the period of the PWM signal in 83.3 us units
 * @param duty3 the desired duty cycle on P2.6 (TA0 CCR3) as a percent (from 0-100)
 * @param duty4 the desired duty cycle on P2.7 (TA0 CCR4) as a percent (from 0-100)
 * @return none
 */
void PWM_Init(uint16_t period, uint16_t duty3, uint16_t duty4) {
    // Set the up down timer to have a period of 83.3 microseconds * period parameter
    upDownConfig.timerPeriod = period * 1000;

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
                                                GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    // Initialize the PWM duty to the parameters duty 3 and 4
    PWM_Duty_Left(duty3);
    PWM_Duty_Right(duty4);

    // Configure Timer_A0 for UpDown Mode
    Timer_A_configureUpDownMode(TIMER_A0_BASE, &upDownConfig);

    // Start TimerA0 in UpDown Mode
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UPDOWN_MODE);

    // Initialize compare registers to generate PWM3
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM3);

    // Initialize compare registers to generate PWM4
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM4);
}

// converts Duty cycle from 0-100 to a CCR value
uint16_t Duty_To_CCR(uint16_t duty) {
    // take the inverse of the duty, and then scale it to the timer period and pass as the compare control for use as the on/off cycle per unit up/down cycle
    return (((100-duty)*upDownConfig.timerPeriod)/100);
}

/**
 * Sets the duty cycle of the PWM output on P2.6 (right motor)
 * @param duty the desired duty cycle on P2.6 (changes TA0 CCR3)
 * @return none
 */
void PWM_Duty_Right(uint16_t duty) {
    // Convert Duty Cycle into the appropriate CCR Value
    uint16_t CCR_Value = Duty_To_CCR(duty);

    // Modify the CCR Value in the configuration
    compareConfig_PWM3.compareValue = CCR_Value;

    // Initialize the timer with the modified configuration
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM3);
}


/**
 * Sets the duty cycle of the PWM output on P2.7 (left motor)
 * @param duty the desired duty cycle on P2.7 (changes TA0 CCR4)
 * @return none
 */
void PWM_Duty_Left(uint16_t duty) {
    // Convert Duty Cycle into the appropriate CCR Value
    uint16_t CCR_Value = Duty_To_CCR(duty);

    // Modify the CCR Value in the configuration
    compareConfig_PWM4.compareValue = CCR_Value;

    // Initialize the timer with the modified configuration
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM4);
}
