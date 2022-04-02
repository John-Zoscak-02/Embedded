/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "Clock.h"
#include "TimerA1.h"
#include "UART0.h"
#include "LaunchPad.h"
#include "Reflectance.h"
#include "Bump.h"
#include "PWM.h"
#include "FSM.h"

volatile uint8_t counter;
volatile uint8_t reflectance_input;
volatile uint8_t bump_input;
volatile uint8_t launchpad;

// Task to be called on a periodic basis by timer a1
void Task() {
//    UART0_OutString("In Task \n\r");
    counter++;
    // on every tenth call of task, call reflectance start
    if (counter == 10) {
        Reflectance_Start();
    }
    // on every tenth call of task (delayed by 1), call reflectance end and the position global variable to it
    else if (counter == 11) {
        reflectance_input = Reflectance_End();
    }
    // on every tenth call of task (delayed by 2), call bump read and set the bump read global variable to it
    // reset counter to two, this will ensure that ten cycles passed between the recent reflectance start and the next one
    else if (counter >= 12) {
        counter = 2;
        bump_input = Bump_Read();
    }
}

 int main(void) {
    FSMType Line_Center_FSM;

    // Counter, the global position variable inits to zero, the global bump variable inits to zero
    counter = 0;
    reflectance_input = 0x00;
    bump_input = 0x00;
    launchpad = 0x00;

    // Initialize clock
    WDT_A_holdTimer();
    Clock_Init();
    PWM_Init(1, 0, 0);
    TimerA1_Init(&Task, 500);

    // Initialize the FSM
    InitializeFSM(&Line_Center_FSM);
    UART0_Init();

    while(1){
        // Whatever this file's global bump input variable is, give that to the FSMs BumpInput
        Line_Center_FSM.BumpInput = bump_input;
        // Whatever this file's global position input variable is, give that to the FSMs BumpInput
        Line_Center_FSM.Position = reflectance_input;
//        reflectance_input = Reflectance_Read(1000);
//        UART0_OutString("Got Here \n\r");
        Line_Center_FSM.CurrentState = NextStateFunction(&Line_Center_FSM);

        launchpad = LaunchPad_Input();
        Line_Center_FSM.LaunchPadInput = launchpad;

        //This code below is capable of decoding the ordinals of the FSM enumerations types into strings, and then uses that to print the current state to the terminal
        switch (Line_Center_FSM.CurrentState) {
            case 0:
                UART0_OutString("Inactive\n\r");
                break;
            case 1:
                UART0_OutString("Bumped\n\r");
                break;
            case 2:
                UART0_OutString("PosLeft\n\r");
                break;
            case 3:
                UART0_OutString("PosRight\n\r");
                break;
            case 4:
                UART0_OutString("PosCenter\n\r");
                break;
            case 5:
                UART0_OutString("PosLost\n\r");
                break;
            default:
                UART0_OutString("Invalid State\n\r");
                break;
        }

        OutputFunction(&Line_Center_FSM);
    }
}
