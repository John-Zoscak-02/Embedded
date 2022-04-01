/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>

/* Project Includes */
#include "RGB_PWM.h"

/* Application Defines  */
#define TIMERA1_PERIOD 127


/* Timer_A UpDown Configuration Parameter */
const Timer_A_UpDownModeConfig upDownConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,               // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // ACLK/1 = 32kHz
        TIMERA1_PERIOD,                         // 127 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};


/* Timer_A Compare Configuration Parameter  (PWM1) */
Timer_A_CompareModeConfig compareConfig_PWM1 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR1
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_SET,              // Toggle output bit
        TIMERA1_PERIOD                              // Initial 0% Duty Cycle
};

/* Timer_A Compare Configuration Parameter (PWM2) */
Timer_A_CompareModeConfig compareConfig_PWM2 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_2,          // Use CCR2
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_SET,              // Toggle output bit
        TIMERA1_PERIOD                              // Initial 0% Duty Cycle
};

/* Timer_A Compare Configuration Parameter (PWM3) */
Timer_A_CompareModeConfig compareConfig_PWM3 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_3,          // Use CCR3
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_SET,              // Toggle output bit
        TIMERA1_PERIOD                              // Initial 0% Duty Cycle
};


// Redefined Port Mapping to be set on Port 2
const uint8_t port_mapping[] =
{
 PMAP_TA1CCR1A, PMAP_TA1CCR2A, PMAP_TA1CCR3A, PMAP_NONE, PMAP_NONE, PMAP_NONE, // Maps TimerA1 CCR outputs to RGB LEDs
        PMAP_NONE, PMAP_NONE
};


// Configures TimerA1 to output PWM Signals to the RGB LEDs
// Duty Cycle is initially set to have a value of zero
void LED_PWM_Init(void) {
    // Reconfigure the port mapping of the CCRs to map to the LEDs
    PMAP_configurePorts((const uint8_t *) port_mapping, PMAP_P2MAP, 1,
                                PMAP_DISABLE_RECONFIGURATION);

    // Set RGB LEDs for CCR Output
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
                GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configure Timer_A1 for UpDown Mode
    Timer_A_configureUpDownMode(TIMER_A1_BASE, &upDownConfig);

    // Start TimerA1 in UpDown Mode
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UPDOWN_MODE);

    // Initialize compare registers to generate PWM1
    Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM1);

    // Initialize compare registers to generate PWM2
    Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM2);

    // Initialize compare registers to generate PWM3
    Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM3);
}


// converts Duty cycle from 0-100 to a CCR value
uint16_t Duty_To_CCR(uint16_t duty) {
    return (((100-duty)*127)/100);
}


// Sets the duty cycle of the red LED according to the argument
void Change_Red_Duty(uint16_t duty) {
    // Convert Duty Cycle into the appropriate CCR Value
    uint16_t CCR_Value = Duty_To_CCR(duty);

    // Modify the CCR Value in the configuration
    compareConfig_PWM1.compareValue = CCR_Value;

    // Initialize the timer with the modified configuration
    Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM1);
}

// Toggles the red LED's duty cycle between 0% duty cycle and value of argument
void Toggle_Red_LED(uint16_t duty) {
    // If the LED has a duty cycle of zero, start the LED with provided Duty Cycle
    if (compareConfig_PWM1.compareValue == TIMERA1_PERIOD) {
        Change_Red_Duty(duty);
    }

    // Otherwise set the LED to have a duty cycle of 0, turning it off
    else{
        Change_Red_Duty(0); // 0% duty cycle
    }
}


// Sets the duty cycle of the green LED according to the argument
void Change_Green_Duty(uint16_t duty) {
    // Convert Duty Cycle into the appropriate CCR Value
    uint16_t CCR_Value = Duty_To_CCR(duty);

    // Modify the CCR Value in the configuration
    compareConfig_PWM2.compareValue = CCR_Value;

    // Initialize the timer with the modified configuration
    Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM2);
}

// Toggles the green LED's duty cycle between 0% duty cycle and value of argument
void Toggle_Green_LED(uint16_t duty) {

    // If the LED has a duty cycle of zero, start the LED with provided Duty Cycle
    if (compareConfig_PWM2.compareValue == TIMERA1_PERIOD) {
        Change_Green_Duty(duty);
    }

    // Otherwise set the LED to have a duty cycle of 0, turning it off
    else{
        Change_Green_Duty(0); // 0% duty cycle
    }
}


// Sets the duty cycle of the blue LED according to the argument
void Change_Blue_Duty(uint16_t duty) {
    // Convert Duty Cycle into the appropriate CCR Value
    uint16_t CCR_Value = Duty_To_CCR(duty);

    // Modify the CCR Value in the configuration
    compareConfig_PWM3.compareValue = CCR_Value;

    // Initialize the timer with the modified configuration
    Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM3);
}

// Toggles the blue LED's duty cycle between 0% duty cycle and value of argument
void Toggle_Blue_LED(uint16_t duty) {

    // If the LED has a duty cycle of zero, start the LED with provided Duty Cycle
    if (compareConfig_PWM3.compareValue == TIMERA1_PERIOD) {
        Change_Blue_Duty(duty);
    }

    // Otherwise set the LED to have a duty cycle of 0, turning it off
    else{
        Change_Blue_Duty(0); // 0% duty cycle
    }
}
