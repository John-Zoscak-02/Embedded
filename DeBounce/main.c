#include "msp.h"
#include <stdint.h>
#include "CortexM.h"
#include "Clock.h"

#include "PortPins.h"
#include "LED.h"
#include "pushbutton.h"
#include "TimerA0.h"
#include "Switch.h"
#include "FSM.h"
#include "FSMSwitchDebounce.h"

#define FALSE 0
#define TRUE 1

// Global variables
uint16_t g1msTimer; // global 1 millisecond timer used for debounce

// Function prototypes
void g1msTimerFunction(void);

void main(void)
{
    DebounceSwitchFSMDefine PushButtonS1;
    FSMType LED_Pushbutton_FSM;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    Clock_Init48MHz();

    // Initialize the hardware
    InitializeLEDPortPins();
    InitializePushButtonPortPin();

    // The input, CCR0Value = (desired period)/(2 microseconds).
    // For a desired period = 1 millisecond, CCR0Value = 500.
    InitializeTimerA0(&g1msTimerFunction,500);

    // Initialize the pushbutton S1 active-low switch debounce Finite State Machine (FSM).
    InitializeDebounceSwitchFSM(&PushButtonS1,(uint8_t *) &(PUSHBUTTON_S1_PORT->IN),(uint8_t) PUSHBUTTON_S1_BIT,
            Active, Inactive, ACTIVE_THRESHOLD,INACTIVE_THRESHOLD);

    // Initialize Finite State Machine (FSM) state variables.
    InitializeFSM(&LED_Pushbutton_FSM);

    EnableInterrupts();

    while (TRUE) {

        // Read current input.
        LED_Pushbutton_FSM.CurrentInput = ReadDebouncedSwitchStatus(&PushButtonS1);

        // Next, based on the input value and the current state, determine the next state.
        LED_Pushbutton_FSM.CurrentState = NextStateFunction(&LED_Pushbutton_FSM,&PushButtonS1);

        // Finally, produce the outputs based on the current state.
        OutputFunction(&LED_Pushbutton_FSM,&PushButtonS1);
    }

}

void g1msTimerFunction(void)
{
    g1msTimer++;
}

