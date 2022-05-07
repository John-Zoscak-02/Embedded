/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* Project Includes */
#include "Clock.h"
#include "msp.h"
#include "TimerA1.h"
#include "UART0.h"
#include "LaunchPad.h"
#include "Reflectance.h"
#include "Bump.h"
#include "PWM.h"
#include "Line_FSM.h"
#include "Base_FSM.h"
#include "Nokia5110.h"
#include "Motor.h"
#include "CortexM.h"
//#include "Booster_Pack/opt3001.h"
//#include "Booster_Pack/i2c_driver.h"
//#include "Booster_Pack/bmi160_support.h"
//#include "Booster_Pack/bme280_support.h"
#include <math.h>

volatile uint8_t counter;
volatile uint8_t reflectance_input;
volatile uint8_t bump_input;
volatile uint8_t launchpad;

//struct bmi160_accel_t s_accelXYZ;
//int8_t accel_off_x;
//int8_t accel_off_y;
//int8_t accel_off_z;
//BMI160_RETURN_FUNCTION_TYPE returnValue;
//int returnRslt;

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

// Calculates the number of true outputs from the reflectance input
uint16_t density(uint8_t data) {
    uint16_t density = 0;
    int i = 0;
    for (i = 0; i < 8; i++) {
           // If a pin was 1, then add it to kpos and increment the kbits counter
           if (data & 0x01) {
               density++;
           }
           // Look at the next pin in the next loop
           data = data >> 1;
       }
    return density;
}

// Returns true if at the end of a maze
uint8_t mazeEnd(uint8_t data) {
    return (data == 0b10011001) || (data == 0b10010001) || (data == 0b10001001)
            || (data == 0b00011001) || (data == 0b00110011)
            || (data == 0b10011000) || (data == 0b11001100);
}

// Booster Sensors have been disabled for sanity purposes
// Problems with I2C are pretty unfixable
 int main(void) {
     DisableInterrupts();
     Clock_Init();
     EnableInterrupts();

    PWM_Init(1, 0, 0);
    Motor_Init();
    TimerA1_Init(&Task, 500);
    Nokia5110_Init();

    LineFSMType Line_Center_FSM;
    BaseFSMType Base_FSM;

    // Counter, the global position variable inits to zero, the global bump variable inits to zero
    counter = 0;
    reflectance_input = 0x00;
    bump_input = 0x00;
    launchpad = 0x00;
    uint8_t defer_line_fsm = 0b00;

    // Initialize the FSM
    InitializeBaseFSM(&Base_FSM);
    InitializeLineFSM(&Line_Center_FSM);

    while(1){
        // Take Launchpad input directly and pass it to the FSM
        launchpad = LaunchPad_Input();
        Base_FSM.LaunchPadInput = launchpad;

        // Whatever this file's global bump input variable is, give that to the FSMs BumpInput
        Base_FSM.BumpInput = bump_input;

        Base_FSM.LightLevel = 500;

        // Whatever this file's global position input variable is, give that to the FSMs BumpInput
        Line_Center_FSM.Position = position(reflectance_input);
        // If the reflectance input is all zeros, then we know that
        Line_Center_FSM.Lost = (reflectance_input == 0);
        Line_Center_FSM.Density = density(reflectance_input);
        Line_Center_FSM.MazeEnd = mazeEnd(reflectance_input);

        // Update the Current state of the FSM to the output of the next state that it should be in
        Base_FSM.CurrentState = BaseNextStateFunction(&Base_FSM);
        Line_Center_FSM.CurrentState = LineNextStateFunction(&Line_Center_FSM, defer_line_fsm);

        // Print the density of the reflectance input and the current reflectance input
        Nokia5110_Clear();
        Nokia5110_SetCursor(0, 2);
        density(reflectance_input) ? Nokia5110_OutUDec(density(reflectance_input)) : Nokia5110_OutChar('0');
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
        defer_line_fsm = BaseOutputFunction(&Base_FSM);
        LineOutputFunction(&Line_Center_FSM, defer_line_fsm);
    }
}
