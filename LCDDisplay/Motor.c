#include "driverlib.h"
#include "PWM.h"
#include "Motor.h"

//   Left Motor:                Right Motor:
//    - Direction: P5.4         - Direction: P5.5
//    - PWM:       P2.7         - PWM        P2.6
//    - Enable:    P3.7         - Enable     P3.6
// initialize PWM outputs to 0% duty cycle on the two motor PWM pins (P2.6 and P2.7)
// initialize motor enable and direction pins as GPIO outputs
// set motors to sleep mode
// set motor direction to forward initially
void Motor_Init(void){
    PWM_Duty_Left(0);
    PWM_Duty_Right(0);

    // Initialize enable ports and pins
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);

    // Initialize direction ports and pins
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4 | GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4 | GPIO_PIN5);
}


// Drive both motors forwards at the given duty cycles
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty){
    // Pass in new duty percentages for the PWM
    PWM_Duty_Left(leftDuty);
    PWM_Duty_Right(rightDuty);

    // Set both motor outputs to high
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
    // Set both motors to go forward
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4 | GPIO_PIN5);
}


// Drive both motors backwards at the given duty cycles
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty){
    // Pass in new duty percentages for the PWM
    PWM_Duty_Left(leftDuty);
    PWM_Duty_Right(rightDuty);

    // Set both motor outputs to high
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
    // Set both motors to go backwards
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4 | GPIO_PIN5);
}


// Drive the right motor forwards and the left motor backwards at the given duty cycles
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty){
    // Pass in new duty percentages for the PWM
    PWM_Duty_Left(leftDuty);
    PWM_Duty_Right(rightDuty);

    // Set both motor outputs to high
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
    // Set the right motor to forward
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);
    // Set the left motor to backwards
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
}


// Drive the right motor backwards and the left motor forwards at the given duty cycles
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty){
    // Pass in new duty percentages for the PWM
    PWM_Duty_Left(leftDuty);
    PWM_Duty_Right(rightDuty);

    // Set both motors outputs to high
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
    // Set the right motor to backwards
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5);
    // Set the left motor to forward
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
}


// Stop the motors and enter sleep mode
void Motor_Stop(void){
    // Pass in a duty percentage of 0 to stop motion of motors
    PWM_Duty_Left(0);
    PWM_Duty_Right(0);

    // Go into sleep mode for the motors
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7);
}
