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
#include "Nokia5110.h"
#include "Bump.h"
#include "PWM.h"
#include "FSM.h"

volatile uint8_t counter;
volatile uint8_t reflectance_input;
volatile uint8_t bump_input;
volatile uint8_t launchpad;

// bitmap of an enemy from Space Invaders
const uint8_t Enemy[] = {
                         0x42, 0x4D, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80,
                         0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF,
                         0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F,
                         0x0F, 0x00, 0x00, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
                         0xFF, 0x0F, 0xF0, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF
};

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

int16_t position(uint8_t data) {
   signed short WEIGHTS[] = {334, 238, 142, 48, -48, -142, -238, -334};
   // Declare kbits to count the number of bits that were 1
   uint8_t kbits = 0;
   // Declare kpos for arithmetic to determine the displacement from the center
   int32_t kpos = 0;
   uint8_t i = 0;
   for (i = 0; i < 8; i++) {
       // If a pin was 1, then add it to kpos and increment the kbits counter
       if (data & 0x01) {
           kpos += WEIGHTS[i];
           kbits++;
       }
       // Look at the next pin in the next loop
       data = data >> 1;
   }
   // return average of all of the weights of the active pins
   return kpos / kbits;
}


FSMState previousState;
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
    Nokia5110_Init();

    // Initialize the FSM
    InitializeFSM(&Line_Center_FSM);

    while(1){
        // Take Launchpad input directly and pass it to the FSM
        launchpad = LaunchPad_Input();
        Line_Center_FSM.LaunchPadInput = launchpad;
        // Whatever this file's global bump input variable is, give that to the FSMs BumpInput
        Line_Center_FSM.BumpInput = bump_input;
        // Whatever this file's global position input variable is, give that to the FSMs BumpInput
        Line_Center_FSM.Position = position(reflectance_input);
        // If the reflectance input is all zeros, then we know that
        Line_Center_FSM.Lost = (reflectance_input == 0);
        // Update the Current state of the FSM to the output of the next state that it should be in
        Line_Center_FSM.CurrentState = NextStateFunction(&Line_Center_FSM);
        Line_Center_FSM.PreviousState = Line_Center_FSM.PreviousState;

        // This will print out the reflectance input to the LCD screen by printing out each bit in the appropriate location in succession.
        Nokia5110_Clear();
        Nokia5110_SetCursor(0, 1);
        Nokia5110_OutChar((reflectance_input &0b10000000) ? '1' : '0');
        Nokia5110_SetCursor(1, 1);
        Nokia5110_OutChar((reflectance_input &0b01000000) ? '1' : '0');
        Nokia5110_SetCursor(2, 1);
        Nokia5110_OutChar((reflectance_input &0b00100000) ? '1' : '0');
        Nokia5110_SetCursor(3, 1);
        Nokia5110_OutChar((reflectance_input &0b00010000) ? '1' : '0');
        Nokia5110_SetCursor(4, 1);
        Nokia5110_OutChar((reflectance_input &0b00001000) ? '1' : '0');
        Nokia5110_SetCursor(5, 1);
        Nokia5110_OutChar((reflectance_input &0b00000100) ? '1' : '0');
        Nokia5110_SetCursor(6, 1);
        Nokia5110_OutChar((reflectance_input &0b00000010) ? '1' : '0');
        Nokia5110_SetCursor(7, 1);
        Nokia5110_OutChar((reflectance_input &0b00000001) ? '1' : '0');

        // Let the FSM do things based on it's current state
        OutputFunction(&Line_Center_FSM);
    }
}