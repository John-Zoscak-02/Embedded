#ifndef RGB_PWM_H_
#define RGB_PWM_H_
#include <stdint.h>


// Configures TimerA1 to output PWM Signals to the RGB LEDs
// Duty Cycle is initially set to have a value of zero
void LED_PWM_Init(void);


// converts Duty cycle from 0-100 to a CCR value
uint16_t Duty_To_CCR(uint16_t duty);


// Sets the duty cycle of the red LED according to the argument
void Change_Red_Duty(uint16_t duty);

// Toggles the red LED's duty cycle between 0% duty cycle and value of argument
void Toggle_Red_LED(uint16_t duty);


// Sets the duty cycle of the green LED according to the argument
void Change_Green_Duty(uint16_t duty);

// Toggles the green LED's duty cycle between 0% duty cycle and value of argument
void Toggle_Green_LED(uint16_t duty);


// Sets the duty cycle of the blue LED according to the argument
void Change_Blue_Duty(uint16_t duty);

// Toggles the blue LED's duty cycle between 0% duty cycle and value of argument
void Toggle_Blue_LED(uint16_t duty);


#endif /* RGB_PWM_H_ */
