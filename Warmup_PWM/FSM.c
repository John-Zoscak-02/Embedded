#include "driverlib.h"
#include "FSM.h"
#include "LaunchPad.h"
#include "Reflectance.h"
#include "Bump.h"
#include "UART0.h"

//--------------------------------------------------------------------------
// Initialize FSM
//--------------------------------------------------------------------------

void InitializeFSM(FSMType *FSM)
{
    // Initialize to the Inactive state when the robot starts and set the turn and brake signals as output pins
    FSM->CurrentState = Inactive;
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);

    // Initialize dependent robot systems
    LaunchPad_Init();
    Reflectance_Init();
    Bump_Init();
}

//--------------------------------------------------------------------------
// Determine next FSM state
//--------------------------------------------------------------------------
FSMState NextStateFunction(FSMType *FSM)
{
    // Redundantly Set the next state to current state
    FSMState NextState = FSM->CurrentState;

    // Decode the current state, look into the current inputs
    switch (FSM->CurrentState){
        case Inactive:
            // Push buttons have highest priority
            if (LaunchPad_Input() & 0x01) {
                if (FSM->Position == 0b11) {
                    NextState = PosCenter;
                }
                else if (FSM->Position == 0b10) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b01) {
                    NextState = PosRight;
                }
                else if (FSM->Position == 0b00) {
                    NextState = PosLost;
                }
            }
            // if the push button Port 1 Pin 1 is off, stay in the inactive state
            else {
//                UART0_OutString("Staying Inactive \n\r");
                NextState = Inactive;
            }
            break;
        case Bumped:
            // Push button Port 1 Pin 4 takes highest priority
            if (LaunchPad_Input() & 0x10) {
                NextState = Inactive;
            }
            // If no longer bumped, look at reflectance to decide the next positional state
            else if (FSM->BumpInput == 0) {
                if (FSM->Position == 0b11) {
                    NextState = PosCenter;
                }
                else if (FSM->Position == 0b10) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b01) {
                    NextState = PosRight;
                }
                else if (FSM->Position == 0b00) {
                    NextState = PosLost;
                }
            }
            // still bumped, stay in bumped
            else {
                NextState = Bumped;
            }
            break;
        case PosLeft:
            // Push button Port 1 Pin 4 takes highest priority
            if (LaunchPad_Input() & 0x10) {
                NextState = Inactive;
            }
            // If bumped, go into the bumped state
            else if (FSM->BumpInput != 0) {
                NextState = Bumped;
            }
            // No push button and also no bump, decide which positional state is next by ReflectanceCenter()
            else {
                if (FSM->Position == 0b11) {
                    NextState = PosCenter;
                }
                else if (FSM->Position == 0b10) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b01) {
                    NextState = PosRight;
                }
                else if (FSM->Position == 0b00) {
                    NextState = PosLost;
                }
            }
            break;
        case PosRight:
            // Push button Port 1 Pin 4 takes highest priority
            if (LaunchPad_Input() & 0x10) {
                NextState = Inactive;
            }
            // If bumped, go into the bumped state
            else if (FSM->BumpInput != 0) {
                NextState = Bumped;
            }
            // No push button and also no bump, decide which positional state is next by ReflectanceCenter()
            else {
                if (FSM->Position == 0b11) {
                    NextState = PosCenter;
                }
                else if (FSM->Position == 0b10) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b01) {
                    NextState = PosRight;
                }
                else if (FSM->Position == 0b00) {
                    NextState = PosLost;
                }
            }
            break;
        case PosCenter:
            // Push button Port 1 Pin 4 takes highest priority
            if (LaunchPad_Input() & 0x10) {
                NextState = Inactive;
            }
            // If bumped, go into the bumped state
            else if (FSM->BumpInput != 0) {
                NextState = Bumped;
            }
            // No push button and also no bump, decide which positional state is next by ReflectanceCenter()
            else {
                if (FSM->Position == 0b11) {
                    NextState = PosCenter;
                }
                else if (FSM->Position == 0b10) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b01) {
                    NextState = PosRight;
                }
                else if (FSM->Position == 0b00) {
                    NextState = PosLost;
                }
            }
            break;
        case PosLost:
            // Push button Port 1 Pin 4 takes highest priority
            if (LaunchPad_Input() & 0x10) {
                NextState = Inactive;
            }
            // If bumped, go into the bumped state
            else if (FSM->BumpInput != 0) {
                NextState = Bumped;
            }
            // No push button and also no bump, decide which positional state is next by ReflectanceCenter()
            else {
                if (FSM->Position == 0b11) {
                    NextState = PosCenter;
                }
                else if (FSM->Position == 0b10) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b01) {
                    NextState = PosRight;
                }
                else if (FSM->Position == 0b00) {
                    NextState = PosLost;
                }
            }
            break;
    }
    return NextState;
}

//--------------------------------------------------------------------------
// Determine LED output based on state
//--------------------------------------------------------------------------
void OutputFunction(FSMType *FSM)
{
    switch (FSM->CurrentState) {
        // If in the inactive state, no lights are on
        case Inactive:
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            break;
        // If in the bumped state, all leds are on
        case Bumped:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            break;
        // If left of line, turn on the right turn signal
        case PosLeft:
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            break;
        // If right of the line, turn on the left turn signal
        case PosRight:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            break;
        // If on the center of the line, turn on both turn signals
        case PosCenter:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            break;
        // If reflectance does not find the line, turn on brake lights
        case PosLost:
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            break;
    }

}
