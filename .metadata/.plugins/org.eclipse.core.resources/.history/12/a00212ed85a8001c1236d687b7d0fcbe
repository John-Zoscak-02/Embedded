//--------------------------------------------------------------------------
// Includes
//--------------------------------------------------------------------------
#include <stdint.h>
#include "driverlib.h"
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

    //Initialize UART
//       UART0_Init();

    //Warm-up: Uncomment the line below for module 0 warm up!
    //ByteValue = WarmUpWithPointers();

    while (TRUE)
    {
        //Read current input
        LED_PushButton_FSM.CurrentInputS1 = ReadSwitchStatus(&PushButtonS1);
        LED_PushButton_FSM.CurrentInputS2 = ReadSwitchStatus(&PushButtonS2);

        //Using input value and current state, determine next state
        LED_PushButton_FSM.CurrentState = NextStateFunction(&LED_PushButton_FSM);

//        switch (LED_PushButton_FSM.CurrentState) {
//            case 0:
//                UART0_OutString("LEDHeartBeatStart\n\r");
//                break;
//            case 1:
//                UART0_OutString("LEDHeartBeat\n\r");
//                break;
//            case 2:
//                UART0_OutString("LEDWaitingToBeat\n\r");
//                break;
//            case 3:
//                UART0_OutString("LEDPauseHeartBeat\n\r");
//                break;
//            case 4:
//                UART0_OutString("LEDColorCyclingStart\n\r");
//                break;
//            case 5:
//                UART0_OutString("LEDSignalsWait\n\r");
//                break;
//            case 6:
//                UART0_OutString("LEDSignals\n\r");
//                break;
//            case 7:
//                UART0_OutString("LEDColorCycling\n\r");
//                break;
//            case 8:
//                UART0_OutString("LEDWaitingToCycle\n\r");
//                break;
//            case 9:
//                UART0_OutString("LEDPauseCycle\n\r");
//                break;
//            case 10:
//                UART0_OutString("LEDStop\n\r");
//                break;
//            default:
//                UART0_OutString("Invalid State\n\r");
//                break;
//
//        }

        //Produce outputs based on current state
        OutputFunction(&LED_PushButton_FSM);
    }
}
