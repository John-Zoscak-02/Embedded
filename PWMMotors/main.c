/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>

/* Project Includes */
#include "PWM.h"
#include "Clock.h"
#include "Motor.h"
#include "Launchpad.h"


uint8_t data;

int main(void)  {

    // Stop watchdog timer
    WDT_A_holdTimer();

    // Call Appropriate Initializations
    Clock_Init();
    LaunchPad_Init();
    PWM_Init(1, 0, 0);
    Motor_Init();

    // Continuously Check SW1 and SW2 and set Duty Cycle According to their value
    while(1){
        // The following code is for controlling the motors with the switch buttons
//        data = LaunchPad_Input();
//        // If no switch button is being pressed, do not turn the motors
//        if (data == 0) {
//            Motor_Stop();
//        }
//        // If the p1 switch button is pressed, turn right
//        else if (data == 1) {
//            Motor_Right(10, 10);
//        }
//        // If the p4 switch button is pressed, turn left
//        else if (data == 2) {
//            Motor_Left(10, 10);
//        }
//        // If the both switch buttons are pressed, go straight
//        else if (data == 3) {
//            Motor_Forward(10, 10);
//        }
        // The following code is for repeating a motor cycle
        data = LaunchPad_Input();
        if (data == 1) {
            Motor_Forward(20, 20);
            Clock_Delay1ms(1000);
            Motor_Right(20, 20);
            Motor_Forward(20, 20);
            Clock_Delay1ms(1000);
            Motor_Right(20, 20);
            Motor_Forward(20, 20);
            Clock_Delay1ms(1000);
            Motor_Right(20, 20);
            Clock_Delay1ms(1000);
            Motor_Backward(20, 20);
            Clock_Delay1ms(1000);
            Motor_Left(20, 20);
            Clock_Delay1ms(1000);
            Motor_Backward(20, 20);
            Clock_Delay1ms(1000);
            Motor_Left(20, 20);
            Clock_Delay1ms(1000);
            Motor_Backward(20, 20);
            Clock_Delay1ms(1000);
            Motor_Left(20, 20);
            Clock_Delay1ms(1000);
            Motor_Stop();
            Clock_Delay1ms(1000);
        }
    }
}
