/* DriverLib Includes */
#include "driverlib.h"
#include "msp.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "Bump.h"

// Define statements for bump switch pins
#define BUMP_PORT   GPIO_PORT_P4
#define BUMP0       GPIO_PIN0   // P4.0
#define BUMP1       GPIO_PIN2   // P4.2
#define BUMP2       GPIO_PIN3   // P4.3
#define BUMP3       GPIO_PIN5   // P4.5
#define BUMP4       GPIO_PIN6   // P4.6
#define BUMP5       GPIO_PIN7   // P4.7
#define BUMP_PINS   (BUMP0 | BUMP1 | BUMP2| BUMP3| BUMP4 | BUMP5)

// Initialize the bump switch pins as GPIO inputs with pull up resistors
// Switches are active low
void Bump_Init(void){
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, BUMP_PINS);
}

// reads values of bump switches
//gives result with positive logic
uint8_t Bump_Read(void){
    // Read in P4
    uint8_t in = ~ P4->IN;
    // Use bit-shift and bit-masking to push together pin0, pin2, pin3, pin5, pin6, and pin7
    uint8_t read =  (0xE0 & in)>>2 | (0x0C & in)>>1 | (0x01 & in);
    // return read in
    return read;
}
