#include "driverlib.h"
#include "FSM.h"
#include "LaunchPad.h"
#include "Reflectance.h"
#include "Motor.h"
#include "Bump.h"
#include "UART0.h"

//--------------------------------------------------------------------------
// Initialize FSM
//--------------------------------------------------------------------------

void InitializeFSM(FSMType *FSM)
{
    // Initialize to the Inactive state when the robot starts and set the turn and brake signals as output pins
    FSM->CurrentState = Inactive;

    // Initialize dependent robot systems
    LaunchPad_Init();
    Reflectance_Init();
    Bump_Init();
    Motor_Init();
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
            if (FSM->LaunchPadInput & 0b01) {
                if (FSM->Position == 0b11) {
                    NextState = PosCenter;
                }
                else if (FSM->Position == 0b01) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b10) {
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
            if (FSM->LaunchPadInput & 0b10) {
                NextState = Inactive;
            }
            // If no longer bumped, look at reflectance to decide the next positional state
            else if (FSM->BumpInput == 0) {
                if (FSM->Position == 0b11) {
                    NextState = PosCenter;
                }
                else if (FSM->Position == 0b01) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b10) {
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
        case PosLeft1:
            // Push button Port 1 Pin 4 takes highest priority
            if (FSM->LaunchPadInput & 0b10) {
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
                else if (FSM->Position == 0b01) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b10) {
                    NextState = PosRight;
                }
                else if (FSM->Position == 0b00) {
                    NextState = PosLost;
                }
            }
            break;
        case PosLeft2:
            // Push button Port 1 Pin 4 takes highest priority
            if (FSM->LaunchPadInput & 0b10) {
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
                else if (FSM->Position == 0b01) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b10) {
                    NextState = PosRight;
                }
                else if (FSM->Position == 0b00) {
                    NextState = PosLost;
                }
            }
            break;
        case PosRight:
            // Push button Port 1 Pin 4 takes highest priority
            if (FSM->LaunchPadInput & 0b10) {
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
                else if (FSM->Position == 0b01) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b10) {
                    NextState = PosRight;
                }
                else if (FSM->Position == 0b00) {
                    NextState = PosLost;
                }
            }
            break;
        case PosCenter:
            // Push button Port 1 Pin 4 takes highest priority
            if (FSM->LaunchPadInput & 0b10) {
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
                else if (FSM->Position == 0b01) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b10) {
                    NextState = PosRight;
                }
                else if (FSM->Position == 0b00) {
                    NextState = PosLost;
                }
            }
            break;
        case PosLost:
            // Push button Port 1 Pin 4 takes highest priority
            if (FSM->LaunchPadInput & 0b10) {
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
                else if (FSM->Position == 0b01) {
                    NextState = PosLeft;
                }
                else if (FSM->Position == 0b10) {
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
        // If in the inactive state, stop
        case Inactive:
            Motor_Stop();
            break;
        // If in the bumped state, stop
        case Bumped:
            Motor_Stop();
            break;
        // If left of line, go right
        case PosLeft1:
            Motor_Right(0, 20);
            break;
        // If right of the line, go left
        case PosRight1:
            Motor_Left(20, 0);
            break;
        // If on the center of the line, go straight
        case PosCenter:
            Motor_Forward(20, 20);
            break;
        // If Reflectance does not find the line, Move backwards a little
        case PosLost:
            Motor_Backward(20, 20);
            break;
    }

}
