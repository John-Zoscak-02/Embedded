#include "driverlib.h"

#include "LaunchPad.h"

//------------LaunchPad_Init------------
// Initialize Switch input
void LaunchPad_Init(void){
  // setup push switches as input with pull up resistor
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
}


//------------LaunchPad_Input------------
// Input from Switches 
// Input: none
// Output: 0x00 none
//         0x01 Button1
//         0x02 Button2
//         0x03 both Button1 and Button2
uint8_t LaunchPad_Input(void){
  // read pin P1.1 [000000X0] and pin P1.4 [000X0000]
  // use bit-shifting to return appropriate output
    uint8_t p1p1 = 0x01 & ~GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
    uint8_t p1p4 = 0x01 & ~GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4);
    return (p1p1) | (p1p4<<1);
}
