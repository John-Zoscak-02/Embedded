/* Driver Library Includes */
#include "gpio.h"
#include "timer_a.h"
#include "interrupt.h"
/* Standard Includes */
#include <stdint.h>
#include "msp.h"
/* Project Includes */
#include "Clock.h"
/* Defines */
volatile uint32_t mclkFreq, smclkFreq;

Timer_A_UpModeConfig config =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_64,
 1,
 TIMER_A_TAIE_INTERRUPT_DISABLE,
 TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
 TIMER_A_DO_CLEAR
};

int main(void){
    Clock_Init();
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);         // Configure P1.0 LED for GPIO output
    Timer_A_configureUpMode(TIMER_A0_BASE, &config);      // Configure timer A0 to up mode
    Interrupt_enableInterrupt(INT_TA0_0);                 // Enable overflow interrupt on Timer A0 CCR0
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE); // Start timer A0
    while(1){
        // Create expressions to see if you
        // Configured system clock as expected
        mclkFreq  = CS_getMCLK();
        smclkFreq = CS_getSMCLK();
    }
}

void TA0_0_IRQHandler(void){
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
}
