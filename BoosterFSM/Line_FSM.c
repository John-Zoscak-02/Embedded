#include "Line_FSM.h"
#include "driverlib.h"
#include "Reflectance.h"
#include "Motor.h"
#include "Clock.h"
#include "Nokia5110.h"

#define FULL_MOTOR 30
#define HALF_MOTOR 15

//--------------------------------------------------------------------------
// Initialize FSM
//--------------------------------------------------------------------------

void InitializeLineFSM(LineFSMType *FSM)
{
    // Initialize to the Inactive state when the robot starts and set the turn and brake signals as output pins
    FSM->CurrentState = Center;
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);

    // Initialize dependent robot systems
    Reflectance_Init();
}

//--------------------------------------------------------------------------
// Determine next FSM state
//--------------------------------------------------------------------------
LineFSMState LineNextStateFunction(LineFSMType *FSM, uint8_t active) {
    // Redundantly Set the next state to current state
    LineFSMState NextState = FSM->CurrentState;
    if (!active) {
        return NextState;
    }
    // Decode the current state, look into the current inputs
    switch (FSM->CurrentState) {
        case PivotLeft:
            if (-334 <= FSM->Position && FSM->Position <= -239) {
                NextState = PivotLeft;
            }
            else if (-238 <= FSM->Position && FSM->Position <= -143) {
                NextState = RotateLeft;
            }
            else {
                NextState = Check;
            }
            break;
        case RotateLeft:
            if (-334 <= FSM->Position && FSM->Position <= -239) {
                NextState = PivotLeft;
            }
            else if (-238 <= FSM->Position && FSM->Position <= -143) {
                NextState = RotateLeft;
            }
            else if (-142 <= FSM->Position && FSM->Position <= -48) {
                NextState = ForwardLeft;
            }
            else {
                NextState = Check;
            }
            break;
        case ForwardLeft:
            if (-238 <= FSM->Position && FSM->Position <= -143) {
                NextState = RotateLeft;
            }
            else if (-142 <= FSM->Position && FSM->Position <= -48) {
                NextState = ForwardLeft;
            }
            else if (-48 <= FSM->Position && FSM->Position <= 48) {
                NextState = Center;
            }
            else {
                NextState = Check;
            }
            break;
        case Center:
            // Non-check motor states can only go directly to adjacent motor-states
            if (-142 <= FSM->Position && FSM->Position <= -48) {
                NextState = ForwardLeft;
            }
            else if (-48 <= FSM->Position && FSM->Position <= 48) {
                NextState = Center;
            }
            else if (48 <= FSM->Position && FSM->Position <= 142) {
                NextState = ForwardRight;
            }
            else {
                NextState = Check;
            }
            break;
        case Check:
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

            break;
        case ForwardRight:
            if (-48 <= FSM->Position && FSM->Position <= 48) {
                NextState = Center;
            }
            else if (48 <= FSM->Position && FSM->Position <= 142) {
                NextState = ForwardRight;
            }
            else if (143 <= FSM->Position && FSM->Position <= 238) {
                NextState = RotateRight;
            }
            else {
                NextState = Check;
            }

            break;
        case RotateRight:
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
            break;
        case PivotRight:
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
            break;
        case HardRight:
            if (!FSM->Lost) {
                if (-334 <= FSM->Position && FSM->Position <= -239) {
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
            else {
                NextState = LC;
            }
            break;
        case LC:
            if (!FSM->Lost) {
                if (-334 <= FSM->Position && FSM->Position <= -239) {
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
            else {
                NextState = Lost;
            }
            break;
        case HardLeft:
            if (!FSM->Lost) {
                if (-334 <= FSM->Position && FSM->Position <= -239) {
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
            else {
                NextState = LC;
            }
            break;
        case RC:
            if (!FSM->Lost) {
                if (-334 <= FSM->Position && FSM->Position <= -239) {
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
            else {
                NextState = Lost;
            }
            break;
        case Lost:
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
            break;
    }
    return NextState;
}

//--------------------------------------------------------------------------
// Determine LED output based on state
//--------------------------------------------------------------------------
void LineOutputFunction(LineFSMType *FSM, uint8_t active) {
    if (active != 0) {
        Nokia5110_SetCursor(0, 3);
            switch (FSM->CurrentState) {
                // If in a left turning motor state, turn on the left turn signal and turn off the right turn signal
                case PivotLeft:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Pivot left
        //            Motor_Left(FULL_MOTOR/active, FULL_MOTOR/active);
                    Nokia5110_OutString("PivotLeft");
                    break;
                case RotateLeft:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Slight left pivot
        //            Motor_Left(0, FULL_MOTOR/active);
                    Nokia5110_OutString("RotateLeft");
                    break;
                case ForwardLeft:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Non pivot forward/left
        //            Motor_Forward(HALF_MOTOR, FULL_MOTOR/active);
                    Nokia5110_OutString("ForwardLeft");
                    break;
                // If in forward moving state, turn on both turn signals
                case Center:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // drive straight
        //            Motor_Forward(FULL_MOTOR/active, FULL_MOTOR/active);
                    Nokia5110_OutString("Center");
                    break;
                case Check:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Nokia5110_OutString("Check");
                    break;
                // If in a right moving state, turn on the right turn signal
                case ForwardRight:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Non pivot forward/right
        //            Motor_Forward(FULL_MOTOR/active, HALF_MOTOR);
                    Nokia5110_OutString("ForwardRight");
                    break;
                case RotateRight:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Slight pivot right
        //            Motor_Right(FULL_MOTOR/active, 0);
                    Nokia5110_OutString("RotateRight");
                    break;
                case PivotRight:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Pivot Right
        //            Motor_Right(FULL_MOTOR/active, FULL_MOTOR/active);
                    Nokia5110_OutString("PivotRight");
                    break;
                case HardRight:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
        //            Motor_Right(FULL_MOTOR/active, 0);
                    Clock_Delay1ms(500);
                    Nokia5110_OutString("HardRight");
                    break;
                case LC:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
        //            Motor_Forward(FULL_MOTOR/active, FULL_MOTOR/active);
                    Nokia5110_OutString("LC");
                    Clock_Delay1ms(500);
                    break;
                case HardLeft:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
        //            Motor_Left(0, FULL_MOTOR/active);
                    Nokia5110_OutString("HardLeft");
                    Clock_Delay1ms(500);
                    break;
                case RC:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
        //            Motor_Forward(FULL_MOTOR/active, FULL_MOTOR/active);
                    Nokia5110_OutString("LC");
                    Clock_Delay1ms(500);
                    break;
                case Lost:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
        //            Motor_Backward(HALF_MOTOR/active, HALF_MOTOR/active);
                    Nokia5110_OutString("Lost");
                    break;
            }
    }
}
