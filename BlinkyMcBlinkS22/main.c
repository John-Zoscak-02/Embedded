//--------------------------------------------------------------------------
// Includes
//--------------------------------------------------------------------------
#include <stdint.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "LED.h"
#include "FSM.h"
#include "Clock.h"
#include "Switch.h"
#include "PortPins.h"
#include "PushButton.h"

//--------------------------------------------------------------------------
// Defines
//--------------------------------------------------------------------------
#define FALSE                  0
#define TRUE                   1
#define STOP_WATCHDOG_TIMER   (WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD)

//--------------------------------------------------------------------------
// Global Variables (needed for warm up activity)
//--------------------------------------------------------------------------
char ByteValue;
//--------------------------------------------------------------------------
// Warm-up activity
//--------------------------------------------------------------------------
char WarmUpWithPointers(void)
{
    /*
     * 1. Declare an 8-bit variable and assign it to 0x55
     * 2. Initialize a pointer variable that points to a char
     * 3. Assign the pointer to the address of the 8-bit variable
     * 4. Dereference the pointer and set it equal to 0xAA
     *    ->The 8-bit variable should now be equal to 0xAA because
     *      you just dereferenced the address of the variable that was
     *      stored in the pointer
     * 5. Return the 8-bit variable for verification
     */
    return '0';
}
//--------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------
void main(void)
{
    FSMType LED_PushButton_FSM;
    SwitchDefine PushButtonS1, PushButtonS2;

    STOP_WATCHDOG_TIMER;
    Clock_Init48MHz();

	//Initialize necessary hardware
	InitializeLEDPortPin();
	InitializePushButtonPortPins();

	//Initialize push buttons S1 and S2 to active low,
	//Logic 0  <->  Active   (Switch pressed)
	//Logic 1  <->  Inactive (Switch released)
	InitializeSwitch(&PushButtonS1, (uint8_t *) &(PUSHBUTTON_S1_PORT->IN),
	                         (uint8_t) PUSHBUTTON_S1_BIT, Active, Inactive);
	InitializeSwitch(&PushButtonS2, (uint8_t *) &(PUSHBUTTON_S2_PORT->IN),
	                         (uint8_t) PUSHBUTTON_S2_BIT, Active, Inactive);

	//Initialize Finite State Machine (FSM) state variables
	InitializeFSM(&LED_PushButton_FSM);

	//Warm-up: Uncomment the line below for module 0 warm up!
	//ByteValue = WarmUpWithPointers();

	while (TRUE)
	{
	    //Read current input
	    LED_PushButton_FSM.CurrentInputS1 = ReadSwitchStatus(&PushButtonS1);
	    LED_PushButton_FSM.CurrentInputS2 = ReadSwitchStatus(&PushButtonS2);

	    TURN_ON_LED8_FL;
	    TURN_ON_LED8_FR;
	    TURN_ON_LED8_BR;
	    TURN_ON_LED8_BL;

	    //Using input value and current state, determine next state
	    LED_PushButton_FSM.CurrentState = NextStateFunction(&LED_PushButton_FSM);

	    //Produce outputs based on current state
	    OutputFunction(&LED_PushButton_FSM);
	}
}
