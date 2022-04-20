#include "driverlib.h"
#include "FSM.h"
#include "LaunchPad.h"
#include "Reflectance.h"
#include "Motor.h"
#include "Bump.h"
#include "UART0.h"
#include "Nokia5110.h"

//--------------------------------------------------------------------------
// Initialize FSM
//--------------------------------------------------------------------------

void InitializeFSM(FSMType *FSM)
{
    // Initialize to the Inactive state when the robot starts and set the turn and brake signals as output pins
    FSM->CurrentState = Inactive;
    FSM->PreviousState = Inactive;
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);

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
        case Inactive:
            // Push buttons have highest priority
            if (FSM->LaunchPadInput & 0b01) {
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            // if the push button Port 1 Pin 1 is off, stay in the inactive state
            else {
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
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            // still bumped, stay in bumped
            else {
                NextState = Bumped;
            }
            break;
        case PivotLeft:
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
                // Non-check motor states can only go directly to adjacent motor-states
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else {
                    NextState = CheckPivotLeft;
                }
            }
            break;
        case CheckPivotLeft:
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
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break;
        case RotateLeft:
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
                // Non-check motor states can only go directly to adjacent motor-states
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else {
                    NextState = CheckRotateLeft;
                }

            }
            break;
        case CheckRotateLeft:
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
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break;
        case ForwardLeft:
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
                // Non-check motor states can only go directly to adjacent motor-states
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else {
                    NextState = CheckForwardLeft;
                }
            }
            break;
        case CheckForwardLeft:
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
                // Non-check motor states can only go directly to adjacent motor-states
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break;
        case Center:
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
                // Non-check motor states can only go directly to adjacent motor-states
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else {
                    NextState = CheckCenter;
                }
            }
            break;
        case CheckCenter:
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
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break;
        case ForwardRight:

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
                // Non-check motor states can only go directly to adjacent motor-states
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
            }
            break;
        case CheckForwardRight:
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
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break;
        case RotateRight:
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
                // Non-check motor states can only go directly to adjacent motor-states
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break;
        case CheckRotateRight:
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
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break;
        case PivotRight:
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
                // Non-check motor states can only go directly to adjacent motor-states
                if (FSM->Lost) {
                    NextState = Lost;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break;
        case CheckPivotRight:
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
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break;
        case Lost:
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
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            break;
    }
    FSM->PreviousState = FSM->CurrentState;
    return NextState;
}

//--------------------------------------------------------------------------
// Determine LED output based on state
//--------------------------------------------------------------------------
void OutputFunction(FSMType *FSM) {
    Nokia5110_SetCursor(0, 3);
    switch (FSM->CurrentState) {
        // If in the inactive state, stop, and turn off all of the lights
        case Inactive:
            // No lights in the inactive state
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            // Stop motors when in the inactive state
            Motor_Stop();
            Nokia5110_OutString("Inactive");
            break;
        // If in any left turning motor state, turn on the left turn signal and turn off the right turn signal
        case PivotLeft:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            // Pivot left
            Motor_Left(30, 30);
            Nokia5110_OutString("Pivot L");
            break;
        case CheckPivotLeft:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            Motor_Left(30, 30);
            Nokia5110_OutString("Check PL");
            break;
        case RotateLeft:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            // Slight left pivot
            Motor_Left(0, 30);
            Nokia5110_OutString("Rotate L");
            break;
        case CheckRotateLeft:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            Motor_Left(0, 30);
            Nokia5110_OutString("Check RL");
            break;
        case ForwardLeft:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            // Non pivot forward/left
            Motor_Forward(15, 30);
            Nokia5110_OutString("Forward L");
            break;
        case CheckForwardLeft:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            Motor_Forward(15, 30);
            Nokia5110_OutString("Check FL");
            break;
        // If in forward moving state, turn on both turn signals
        case Center:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            // drive straight
            Motor_Forward(30, 30);
            Nokia5110_OutString("Center");
            break;
        case CheckCenter:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            Motor_Forward(30, 30);
            Nokia5110_OutString("Check C");
            break;
        // If in a right moving state, turn on the right turn signal
        case ForwardRight:
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            // Non pivot forward/right
            Motor_Forward(30, 15);
            Nokia5110_OutString("Forward R");
            break;
        case CheckForwardRight:
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            Motor_Forward(30, 15);
            Nokia5110_OutString("Check FR");
            break;
        case RotateRight:
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            // Slight pivot right
            Motor_Right(30, 0);
            Nokia5110_OutString("Rotate R");
            break;
        case CheckRotateRight:
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            Motor_Right(30, 0);
            Nokia5110_OutString("Check RR");
            break;
        case PivotRight:
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            // Pivot Right
            Motor_Right(30, 30);
            Nokia5110_OutString("Pivot R");
            break;
        case CheckPivotRight:
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
            Motor_Right(30, 30);
            Nokia5110_OutString("Check PR");
            break;
        // If bumped, turn on the brake lights and stop moving
        case Bumped:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            // stop moving
            Motor_Stop();
            Nokia5110_OutString("Bumped");
            break;
        // If the line is lost, move backwards and turn off all the lights
        case Lost:
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            Motor_Backward(30, 30);
            Nokia5110_OutString("Lost");
            break;
    }

}
