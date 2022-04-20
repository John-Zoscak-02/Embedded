#include "Base_FSM.h"
#include "driverlib.h"
#include "LaunchPad.h"
#include "Reflectance.h"
#include "Motor.h"
#include "Bump.h"
#include "Nokia5110.h"

//--------------------------------------------------------------------------
// Initialize FSM
//--------------------------------------------------------------------------

#define LIGHT_OUT_THRESHOLD 100
#define LIGHT_DIM_THRESHOLD 500
#define ACCEL_Z_UP_THRESHOLD 20000
#define ACCEL_Z_STOP_UP_THRESHOLD 10000
#define ACCEL_Z_STABLE_FROM_STOP_THRESHOLD 17000
#define ACCEL_Z_DOWN_THRESHOLD 10000
#define ACCEL_Z_STOP_DOWN_THRESHOLD 20000

void InitializeBaseFSM(BaseFSMType *FSM)
{
    // Initialize to the Inactive state when the robot starts and set the turn and brake signals as output pins
    FSM->CurrentState = Inactive;

    // Initialize dependent robot systems
    LaunchPad_Init();
    Bump_Init();
    Motor_Init();
}

//--------------------------------------------------------------------------
// Determine next FSM state
//--------------------------------------------------------------------------
BaseFSMState BaseNextStateFunction(BaseFSMType *FSM)
{
    // Redundantly Set the next state to current state
    BaseFSMState NextState = FSM->CurrentState;

    // Decode the current state, look into the current inputs
    switch (FSM->CurrentState) {
        case LightOut:
            if (FSM->LaunchPadInput == 2) {
                NextState = Inactive;
            }
            else if (FSM->LightLevel < LIGHT_OUT_THRESHOLD) {
                NextState = LightOut;
            }
            else if (FSM->LightLevel < LIGHT_DIM_THRESHOLD) {
                NextState = Slow;
            }
            else {
                NextState = Normal;
            }
            break;
        case Inactive:
            if (FSM->LaunchPadInput == 1) {
                if (FSM->BumpInput) {
                    NextState = Bumped;
                }
                else if (FSM->LightLevel < LIGHT_OUT_THRESHOLD) {
                    NextState = LightOut;
                }
                else if (FSM->LightLevel < LIGHT_DIM_THRESHOLD) {
                    NextState = Slow;
                }
                else {
                    NextState = Normal;
                }
            }
            break;
        case Normal:
            if (FSM->LaunchPadInput == 2) {
                NextState = Inactive;
            }
            else if (FSM->BumpInput) {
                NextState = Bumped;
            }
            else if (FSM->AccelZ > ACCEL_Z_UP_THRESHOLD) {
                NextState = GoingUp;
            }
            else if (FSM->LightLevel < LIGHT_OUT_THRESHOLD) {
                NextState = LightOut;
            }
            else if (FSM->LightLevel < LIGHT_DIM_THRESHOLD) {
                NextState = Slow;
            }
            else {
                NextState = Normal;
            }
            break;
        case Slow:
            if (FSM->LaunchPadInput == 2) {
                NextState = Inactive;
            }
            else if (FSM->BumpInput) {
                NextState = Bumped;
            }
            else if (FSM->AccelZ > ACCEL_Z_UP_THRESHOLD) {
                NextState = GoingUp;
            }
            else if (FSM->LightLevel < LIGHT_OUT_THRESHOLD) {
                NextState = LightOut;
            }
            else if (FSM->LightLevel < LIGHT_DIM_THRESHOLD) {
                NextState = Slow;
            }
            else {
                NextState = Normal;
            }
            break;
        case Bumped:
            // Non-check motor states can only go directly to adjacent motor-states
            if (FSM->LaunchPadInput == 2) {
                NextState = Inactive;
            }
            else if (FSM->BumpInput) {
                if (FSM->LightLevel < LIGHT_OUT_THRESHOLD) {
                    NextState = LightOut;
                }
                else if (FSM->LightLevel < LIGHT_DIM_THRESHOLD) {
                    NextState = Slow;
                }
                else {
                    NextState = Normal;
                }
            }
            else {
                NextState = Bumped;
            }
            break;
        case GoingUp:
            if (FSM->AccelZ > ACCEL_Z_STOP_UP_THRESHOLD) {
                NextState = StopUp;
            }
            else {
                NextState = GoingUp;
            }
            break;
        case StopUp:
            if (FSM->AccelZ < ACCEL_Z_STABLE_FROM_STOP_THRESHOLD) {
                NextState = PickedUp;
            }
            else {
                NextState = StopUp;
            }
            break;
        case PickedUp:
            if (FSM->AccelZ < ACCEL_Z_DOWN_THRESHOLD) {
                NextState = GoingDown;
            }
            else {
                NextState = PickedUp;
            }
            break;
        case GoingDown:
            if (FSM->AccelZ < ACCEL_Z_STOP_DOWN_THRESHOLD) {
                if (FSM->LightLevel < LIGHT_OUT_THRESHOLD) {
                    NextState = LightOut;
                }
                else if (FSM->LightLevel < LIGHT_DIM_THRESHOLD) {
                    NextState = Slow;
                }
                else {
                    NextState = Normal;
                }
            }
            else {
                NextState = GoingDown;
            }
            break;
    }
    return NextState;
}

//--------------------------------------------------------------------------
// Determine LED output based on state
//--------------------------------------------------------------------------
uint8_t BaseOutputFunction(BaseFSMType *FSM) {
    uint8_t deferControl = 0;
    Nokia5110_SetCursor(0, 3);
    switch (FSM->CurrentState) {
        case LightOut:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("LightOut");
            break;
        case Inactive:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Inactive");
            break;
        case Normal:
            deferControl = 1;
            break;
        case Slow:
            deferControl = 2;
            break;
        case Bumped:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Bumped");
            break;
        case GoingUp:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Put me down!");
            break;
        case StopUp:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Put me down!");
            break;
        case PickedUp:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Put me down!");
            break;
        case GoingDown:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Put me down!");
            break;
    }
    return deferControl;
}
