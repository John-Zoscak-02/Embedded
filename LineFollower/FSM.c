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
    switch (FSM->CurrentState) {
        case Inactive: {
            // Push buttons have highest priority
            if (FSM->LaunchPadInput & 0b01) {
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            // if the push button Port 1 Pin 1 is off, stay in the inactive state
            else {
                NextState = Inactive;
            }
            break; }
        case Bumped: {
            // Push button Port 1 Pin 4 takes highest priority
            if (FSM->LaunchPadInput & 0b10) {
                NextState = Inactive;
            }
            // If no longer bumped, look at reflectance to decide the next positional state
            else if (FSM->BumpInput == 0) {
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            // still bumped, stay in bumped
            else {
                NextState = Bumped;
            }
            break; }
        case PivotLeft: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else {
                    NextState = CheckPivotLeft;
                }
            }
            break; }
        case CheckPivotLeft: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break; }
        case RotateLeft: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else {
                    NextState = CheckRotateLeft;
                }
            }
            break; }
        case CheckRotateLeft: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break; }
        case ForwardLeft: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else {
                    NextState = CheckForwardLeft;
                }
            }
            break; }
        case CheckForwardLeft: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break; }
        case Center: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else {
                    NextState = CheckCenter;
                }
            }
            break; }
        case CheckCenter: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break; }
        case ForwardRight: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
            }
            break; }
        case CheckForwardRight: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break; }
        case RotateRight: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break; }
        case CheckRotateRight: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break; }
        case PivotRight: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break; }
        case CheckPivotRight: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break; }
        case Lost: {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (FSM->BumpInput) {
                    NextState = Bumped;
                }
                else if (-334 <= FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break; }
    }
    return NextState;
}

//--------------------------------------------------------------------------
// Determine LED output based on state
//--------------------------------------------------------------------------
void OutputFunction(FSMType *FSM) {
    switch (FSM->CurrentState) {
        // If in the inactive state, stop
        case Inactive:
            Motor_Stop();
            break;
        case PivotLeft:
            Motor_Left(20, 20);
            break;
        case CheckPivotLeft:
            Motor_Left(20, 20);
            break;
        case RotateLeft:
            Motor_Left(20, 0);
            break;
        case CheckRotateLeft:
            Motor_Left(20, 0);
            break;
        case ForwardLeft:
            Motor_Forward(20, 10);
            break;
        case CheckForwardLeft:
            Motor_Forward(20, 10);
            break;
        case Center:
            Motor_Forward(20, 20);
            break;
        case CheckCenter:
            Motor_Forward(20, 20);
            break;
        case ForwardRight:
            Motor_Forward(10, 20);
            break;
        case CheckForwardRight:
            Motor_Forward(10, 20);
            break;
        case RotateRight:
            Motor_Right(0, 20);
            break;
        case CheckRotateRight:
            Motor_Right(0, 20);
            break;
        case PivotRight:
            Motor_Right(20, 20);
            break;
        case CheckPivotRight:
            Motor_Right(0, 20);
            break;
        case Bumped:
            Motor_Stop();
            break;
        case Lost:
            Motor_Backward(10, 10);
            break;
    }

}