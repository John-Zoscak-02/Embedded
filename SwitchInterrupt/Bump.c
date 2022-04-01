/* DriverLib Includes */
#include "driverlib.h"
#include "msp.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "Bump.h"
#include "UART0.h"

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
    UART0_OutString("Bump init");
    UART0_OutString("\n\r");
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


#define BUMP_INTERRUPT_PRIORITY 0
void (*BumpTask)(uint8_t bumpData); // function pointer for user task when bump interrupt is detected

// Initializes the bump switch pins and enables Port 4 GPIO interrupts
void Bump_Interrupt_Init(void(*task)(uint8_t)){
    //Write this for Interrupt Module Race-Day
    // set BumpTask to be then user function to be called in ISR,
    BumpTask = task;

    // initialize bump pins as GPIO inputs with pull up resistors
    Bump_Init();
    // configure falling edge interrupts on bump pins
    GPIO_interruptEdgeSelect(BUMP_PORT, BUMP_PINS, GPIO_HIGH_TO_LOW_TRANSITION);
    // clear interrupt flags on bump pins
    GPIO_clearInterruptFlag(BUMP_PORT, BUMP_PINS);
    // enable interrupts with GPIO on the bump pins
    GPIO_enableInterrupt(BUMP_PORT, BUMP_PINS);
    // enable the P4 interrupts in the NVIC
    Interrupt_enableInterrupt(INT_PORT4);
    // set the bump interrupts to the desired priority (remember to shift it to the correct location)
    Interrupt_setPriority(INT_PORT4, BUMP_INTERRUPT_PRIORITY << 5);
}


// ISR for bump interrupts
// clear interrupt flag, read bump switches, and call user function for handling a collision
// there is only one line of code for you to add to this function, that is the call to the DL function 
// that clears the interrupt flag.
void PORT4_IRQHandler(void){
    uint8_t bumpData;
    //write this for Interrupt Module Race-Day
    // read bump switch data to be passed to the bump task
    UART0_OutString("Enter the ISR");
    UART0_OutString("\n\r");

    // Read the bump data for passing to task
    bumpData = FSM->BumpInput;

    // Call the bump task with the bumpData
    BumpTask(bumpData);

    // clear interrupt flags
    UART0_OutString("Bump interrupt flags are cleared");
    UART0_OutString("\n\r");
    GPIO_clearInterruptFlag(BUMP_PORT, BUMP_PINS);

}
