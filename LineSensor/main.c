/* Driver Library Includes */
#include "gpio.h"
#include "cs.h"
#include "timer_a.h"
#include "interrupt.h"
/* Standard Includes */
#include <stdint.h>
#include "msp.h"
/* Project Includes */
#include "Clock.h"
#include "LaunchPad.h"
#include "Reflectance.h"
/* Defines */
volatile uint32_t switchInputs;
volatile uint8_t line_sense;
volatile uint8_t line_center;
volatile uint32_t line_pos;


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
    // Initializes the clock to program default
    Clock_Init();
    // Initializes an LED and button program
    LaunchPad_Init();
    // Initializes the polulu IR system for detecting a line below the robot
    Reflectance_Init();
    while(1) {
        // Get the input from P1.1 and P1.2 and add them together, store in switch inputs
        switchInputs = LaunchPad_Input();
        //  LED out put logic: Input: 0 off, bit0=red,bit1=green,bit2=blue
        LaunchPad_LED(switchInputs);
        if (switchInputs == 1) {
            LaunchPad_Output(1);
        } else if (switchInputs == 2) {
            LaunchPad_Output(2);
        } else if (switchInputs == 3) {
            LaunchPad_Output(4);
        }
        else {
            LaunchPad_Output(0);
        }

        // Get which IR lights (pins) are above a black line from the polulu IR lights port
        line_sense = Reflectance_Read(1000);
        // Get relevant center indicators
        line_center = Reflectance_Center(1000);
        // Get the displacement in milimeters from the center of the line.
        line_pos = Reflectance_Position(line_sense);
    }
}
void TA0_0_IRQHandler(void){
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
}


