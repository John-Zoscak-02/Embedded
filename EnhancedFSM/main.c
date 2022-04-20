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
#include "Line_FSM.h"
#include "Base_FSM.h"
#include "Nokia5110.h"
#include "Booster_Pack/opt3001.h"
#include "Booster_Pack/i2c_driver.h"
#include "Booster_Pack/bmi160_support.h"
#include "Booster_Pack/bme280_support.h"

volatile uint8_t counter;
volatile uint8_t reflectance_input;
volatile uint8_t bump_input;
volatile uint8_t launchpad;
volatile uint8_t defer_line_fsm;

struct bmi160_accel_t s_accelXYZ;
int8_t accel_off_x;
int8_t accel_off_y;
int8_t accel_off_z;
BMI160_RETURN_FUNCTION_TYPE returnValue;
int returnRslt;

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

    //Read Accel and Gyro values
    returnValue = bmi160_read_accel_xyz(&s_accelXYZ);
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


 int main(void) {
    LineFSMType Line_Center_FSM;
    BaseFSMType Base_FSM;

    // Counter, the global position variable inits to zero, the global bump variable inits to zero
    counter = 0;
    reflectance_input = 0x00;
    bump_input = 0x00;
    launchpad = 0x00;
    defer_line_fsm = 0b00;
    float convertedLux = 0;

    // Initialize clock
    WDT_A_holdTimer();
    Clock_Init();
    UART0_Init();
    //intialize i2c to communicate with booster pack
    initI2C();
    PWM_Init(1, 0, 0);
    TimerA1_Init(&Task, 500);
    Nokia5110_Init();

    // Initialize the FSM
    InitializeBaseFSM(&Base_FSM);
    InitializeLineFSM(&Line_Center_FSM);

    //Intialize opt3001 sensor
    sensorOpt3001Init();

    //Intialize BMI160 and BMM sensor
    bmi160_initialize_sensor();
    returnRslt = bmi160_config_running_mode(APPLICATION_NAVIGATION); //there are several modes to run the BMI sensor in
    bmi160_accel_foc_trigger_xyz(0x03, 0x03, 0x01, &accel_off_x, &accel_off_y, &accel_off_z);
//    bmi160_set_foc_gyro_enable(0x01, &gyro_off_x, &gyro_off_y, &gyro_off_z);

    while(1){
        // Take Launchpad input directly and pass it to the FSM
        launchpad = LaunchPad_Input();
        Base_FSM.LaunchPadInput = launchpad;
        // Whatever this file's global bump input variable is, give that to the FSMs BumpInput
        Base_FSM.BumpInput = bump_input;
        // Tell the Base FSM the current light level
        uint16_t rawData;
        sensorOpt3001Read(&rawData);
        sensorOpt3001Convert(rawData, &convertedLux);
        Base_FSM.LightLevel = convertedLux;
        // Tell the Base FSM the current Z acceleration
        Base_FSM.AccelZ = s_accelXYZ.z;
        // Whatever this file's global position input variable is, give that to the FSMs BumpInput
        Line_Center_FSM.Position = position(reflectance_input);
        // If the reflectance input is all zeros, then we know that
        Line_Center_FSM.Lost = (reflectance_input == 0);

        // Update the Current state of the FSM to the output of the next state that it should be in
        Base_FSM.CurrentState = BaseNextStateFunction(&Base_FSM);
        Line_Center_FSM.CurrentState = LineNextStateFunction(&Line_Center_FSM, defer_line_fsm);

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
        defer_line_fsm = BaseOutputFunction(&Base_FSM);
        LineOutputFunction(&Line_Center_FSM, defer_line_fsm);
    }
}
