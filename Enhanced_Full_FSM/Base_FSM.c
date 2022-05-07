#include <stdio.h>

#include "Base_FSM.h"
#include "driverlib.h"
#include "LaunchPad.h"
#include "Reflectance.h"
#include "Motor.h"
#include "Bump.h"
#include "Nokia5110.h"
#include "UART0.h"

//--------------------------------------------------------------------------
// Initialize FSM
//--------------------------------------------------------------------------

#define LIGHT_OUT_THRESHOLD                     20
#define LIGHT_DIM_THRESHOLD                     100

#define ACCEL_Z_UP_THRESHOLD                    17000
#define ACCEL_Z_STOP_UP_THRESHOLD               15300
#define ACCEL_Z_STABLE_FROM_UP_THRESHOLD        16300
#define ACCEL_Z_DOWN_THRESHOLD                  15300
#define ACCEL_Z_STOP_DOWN_THRESHOLD             17000
#define ACCEL_Z_STABLE_FROM_DOWN_THRESHOLD      16500

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
            else if (!FSM->BumpInput) {
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
            if (FSM->LaunchPadInput == 2) {
                NextState = Inactive;
            }
            else if (FSM->AccelZ < ACCEL_Z_STOP_UP_THRESHOLD) {
                NextState = StopUp;
            }
            else {
                NextState = GoingUp;
            }
            break;
        case StopUp:
            if (FSM->LaunchPadInput == 2) {
                NextState = Inactive;
            }
            else if (FSM->AccelZ > ACCEL_Z_STABLE_FROM_UP_THRESHOLD) {
                NextState = PickedUp;
            }
            else {
                NextState = StopUp;
            }
            break;
        case PickedUp:
            if (FSM->LaunchPadInput == 2) {
                NextState = Inactive;
            }
            else if (FSM->AccelZ < ACCEL_Z_DOWN_THRESHOLD) {
                NextState = GoingDown;
            }
            else {
                NextState = PickedUp;
            }
            break;
        case GoingDown:
            if (FSM->LaunchPadInput == 2) {
                NextState = Inactive;
            }
            else if (FSM->AccelZ > ACCEL_Z_STOP_DOWN_THRESHOLD) {
                NextState = StoppedDown;
            }
            else {
                NextState = GoingDown;
            }
            break;
        case StoppedDown:
            if (FSM->LaunchPadInput == 2) {
                NextState = Inactive;
            }
            else if (FSM->AccelZ < ACCEL_Z_STABLE_FROM_DOWN_THRESHOLD) {
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
                NextState = StoppedDown;
            }
    }
    return NextState;
}

//--------------------------------------------------------------------------
// Determine LED output based on state
//--------------------------------------------------------------------------
uint8_t BaseOutputFunction(BaseFSMType *FSM) {
    uint8_t deferControl = 0;
    Nokia5110_SetCursor(0, 2);
    switch (FSM->CurrentState) {
        case LightOut:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("LightOut");
//            printf("Light Out\n\r");
            break;
        case Inactive:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Inactive");
//            printf("Inactive\n\r");
            break;
        case Normal:
            deferControl = 1;
            Nokia5110_OutString("Normal");
//            printf("Normal : ");
            break;
        case Slow:
            deferControl = 2;
            Nokia5110_OutString("Slow");
//            printf("Slow : ");
            break;
        case Bumped:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Bumped");
//            printf("Bumped\n\r");
            break;
        case GoingUp:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Put me down!");
//            printf("Going Up\n\r");
            break;
        case StopUp:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Put me down!");
//            printf("Stop Up\n\r");
            break;
        case PickedUp:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Put me down!");
//            printf("Picked Up\n\r");
            break;
        case GoingDown:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Put me down!");
//            printf("Going Down\n\r");
            break;
        case StoppedDown:
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
            deferControl = 0;
            Motor_Stop();
            Nokia5110_OutString("Put me down!");
//            printf("Stopped Down\n\r");
            break;
    }
    return deferControl;
}
