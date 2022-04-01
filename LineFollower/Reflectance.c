#include <stdint.h>
/*
 * Reflectance.c
 * Provides functions to
 * 1. Initialize Pololu reflectance sensor
 * 2. Read Pololu reflectance sensor
 * 3. Determine robot position
 * 4. Determine robot status
 *
 */

#include "driverlib.h"

#include "Clock.h"
#include "Reflectance.h"

#define ALLBITS  0xFF
#define BITSHIFT 0x01

//------------Reflectance_Init------------
// Initialize sensor array to GPIO, set LEDs (P5.3 and P9.2) 
// as output and sensors (P7.0-P7.7) as output
// Input: none
// Output: none
void Reflectance_Init(void){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);

    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
}


//------------Reflectance_Read------------
// Read reflectance sensor
// Input: the delay time in us
// Output: result the sensor readings, bit 0 corresponds 
//         to the rightmost sensor, bit 7 to the leftmost sensor
uint8_t Reflectance_Read(uint32_t time){
    // Delay for 10 milliseconds to ensure that there is no interference with preceding calls of Reflectance_Read
    Clock_Delay1ms(10);
    // 1. Turn on even and odd LEDs
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2);
    // 2. Charge reflectance sensor capacitors (set as output and high on P7.0-P7.7)
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    // 3. Wait 10us for capacitors to charge
    Clock_Delay1us(10);
    // 4. Set reflectance sensor (P7.0-P7.7) as input
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    // 5. Wait @param time
    Clock_Delay1us(time);
    // 6. Read reflectance sensor values and assign to result
    uint8_t result = P7->IN;
    // 8. Turn off even and odd LEDs
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);
    return result;
}


void Reflectance_Start() {
    // 1. Turn on even and odd LEDs
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2);
    // 2. Charge reflectance sensor capacitors (set as output and high on P7.0-P7.7)
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);

    Clock_Delay1us(10);
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
}

uint8_t Reflectance_End() {
    // 6. Read reflectance sensor values and assign to result
    uint8_t result = P7->IN;
    // 8. Turn off even and odd LEDs
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);

    return result;
}


//------------Reflectance_Center------------
// Determine robot's status over the line
// Input: the delay time in us
// Output: result the robot location status (LOST/RIGHT/LEFT/ON LINE)
uint8_t Reflectance_Center(uint32_t time){
    //Get reflectance read data
    uint8_t reflectance_read = Reflectance_Read(time);
    //Get value from left center sensor
    uint8_t left_center = reflectance_read & 0x10;
    //Get value from right center sensor
    uint8_t right_center = reflectance_read & 0x08;
    
    /* Check robot status truth table
     * INPUT (L,R) | OUTPUT
     * ------------|------------
     *      11     | ON_LINE (3)
     *      10     | LEFT    (2)
     *      01     | RIGHT   (1)
     *      00     | LOST    (0)
     */
    if (left_center && right_center) {
        return 0b11;
    }
    else if (left_center && ~right_center) {
       return 0b10;
    }
    else if (~left_center && right_center) {
       return 0b01;
    }
    else {
        return 0b00;
    }
}

//------------Reflectance_Position------------
// Determine robot's status over the line
// Input: the collected sensor data 
// Output: the position value between +345 (most left)
//         to -345 (most right)
int32_t Reflectance_Position(uint8_t data){
    // Declare an array assigning weights to the polulu IR light pins based on thier distance from the center of the robot.
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

