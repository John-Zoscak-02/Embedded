#include "Line_FSM.h"
#include "driverlib.h"
#include "Reflectance.h"
#include "Motor.h"
#include "Clock.h"
#include "Nokia5110.h"
#include "UART0.h"

#define FULL_MOTOR 40
#define HALF_MOTOR 32
//#define FULL_MOTOR 0
//#define HALF_MOTOR 0

#define TURN_90_TIME 500

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

LineFSMState NextPositionState(LineFSMType *FSM) {
    if (FSM->Lost) {
        return Lost;
    }
    else if (-334 <= FSM->Position && FSM->Position <= -239) {
        return PivotLeft;
    }
    else if (-238 <= FSM->Position && FSM->Position <= -143) {
        return RotateLeft;
    }
    else if (-142 <= FSM->Position && FSM->Position <= -48) {
        return ForwardLeft;
    }
    else if (-48 <= FSM->Position && FSM->Position <= 48) {
        return Center;
    }
    else if (48 <= FSM->Position && FSM->Position <= 142) {
        return ForwardRight;
    }
    else if (143 <= FSM->Position && FSM->Position <= 238) {
        return RotateRight;
    }
    else if (239 <= FSM->Position && FSM->Position <= 334) {
        return PivotRight;
    }
    else {
        return FSM->CurrentState;
    }
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
            if (FSM->Lost) {
                NextState = HardRight;
            }
            NextState = NextPositionState(FSM);
            break;
        case RotateLeft:
            if (-143 <= FSM->Position && FSM->Position <= -48) {
                FSM->cross++;
            }
            if (FSM->Position <= -143) {
                FSM->cross=0;
            }
            if (FSM->Lost) {
                NextState = HardRight;
            }
            else {
                NextState = NextPositionState(FSM);
            }
            break;
        case ForwardLeft:
            if (FSM->cross >= 3) {
                NextState = AdjustR;
            }
            else {
                NextState = NextPositionState(FSM);
            }
            break;
        case AdjustL:
            FSM->cross = 0;
            NextState = NextPositionState(FSM);
        case Center:
            // Non-check motor states can only go directly to adjacent motor-states
            NextState = NextPositionState(FSM);
            break;
        case AdjustR:
            FSM->cross = 0;
            NextState = NextPositionState(FSM);
        case ForwardRight:
            if (FSM->cross >= 3) {
                NextState = AdjustL;
            }
            else {
                NextState = NextPositionState(FSM);
            }
            break;
        case RotateRight:
            if (48 <= FSM->Position && FSM->Position <= 143) {
                FSM->cross++;
            }
            else if (FSM->Position >= 143) {
                FSM->cross=0;
            }
            if (FSM->Lost) {
                NextState = HardLeft;
            }
            else {
                NextState = NextPositionState(FSM);
            }
            break;
        case PivotRight:
            if (FSM->Lost) {
                NextState = HardLeft;
            }
            NextState = NextPositionState(FSM);
            break;
        case HardRight:
            if (!FSM->Lost) {
                NextState = NextPositionState(FSM);
            }
            else {
                NextState = LC;
            }
            break;
        case LC:
            if (!FSM->Lost) {
                NextState = NextPositionState(FSM);
            }
            else {
                NextState = Lost;
            }
            break;
        case HardLeft:
            if (!FSM->Lost) {
                NextState = NextPositionState(FSM);
            }
            else {
                NextState = LC;
            }
            break;
        case RC:
            if (!FSM->Lost) {
                NextState = NextPositionState(FSM);
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
    Nokia5110_SetCursor(0, 3);
    if (active != 0) {
            switch (FSM->CurrentState) {
                // If in a left turning motor state, turn on the left turn signal and turn off the right turn signal
                case PivotLeft:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Pivot left
                    Motor_Left(HALF_MOTOR/active, FULL_MOTOR/active);
                    Nokia5110_OutString("PivotLeft");
//                    printf("Pivot Left\n\r");
                    break;
                case RotateLeft:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Slight left pivot
//                    Motor_Forward((HALF_MOTOR/2)/active, FULL_MOTOR/active);
                    Motor_Left((HALF_MOTOR/4)/active, FULL_MOTOR/active);
//                    Motor_Left(0, HALF_MOTOR/active);
//                    Motor_Forward(0, FULL_MOTOR/active);
                    Nokia5110_OutString("RotateLeft");
//                    printf("Rotate Left\n\r");
                    break;
                case ForwardLeft:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Non pivot forward/left
                    Motor_Forward(HALF_MOTOR/active, FULL_MOTOR/active);
                    Nokia5110_OutString("ForwardLeft");
//                    printf("Forward Left\n\r");
                    break;
                // If in forward moving state, turn on both turn signals
                case AdjustL:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Pivot Right
                    Motor_Left(HALF_MOTOR/active, HALF_MOTOR/active);
                    Nokia5110_OutString("AdjustL");
                    Clock_Delay1ms(50);
//                    printf("Pivot Right\n\r");
                case Center:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // drive straight
                    Motor_Forward(FULL_MOTOR/active, FULL_MOTOR/active);
                    Nokia5110_OutString("Center");
//                    printf("Center\n\r");
                    break;
                // If in a right moving state, turn on the right turn signal
                case AdjustR:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Pivot left
                    Motor_Right(HALF_MOTOR/active, FULL_MOTOR/active);
                    Clock_Delay1ms(50);
                    Nokia5110_OutString("AdjustR");
//                    printf("Pivot Left\n\r");
                    break;
                case ForwardRight:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Non pivot forward/right
                    Motor_Forward(FULL_MOTOR/active, HALF_MOTOR/active);
                    Nokia5110_OutString("ForwardRight");
//                    printf("Forward Right\n\r");
                    break;
                case RotateRight:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Slight pivot right
                    Motor_Forward(FULL_MOTOR/active, (HALF_MOTOR/2)/active);
//                    Motor_Right(FULL_MOTOR/active, (HALF_MOTOR/4)/active);
//                    Motor_Right(HALF_MOTOR/active, 0);
//                    Motor_Forward(FULL_MOTOR/active, 0);
                    Nokia5110_OutString("RotateRight");
//                    printf("Rotate Right\n\r");
                    break;
                case PivotRight:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Pivot Right
                    Motor_Right(FULL_MOTOR/active, HALF_MOTOR/active);
                    Nokia5110_OutString("PivotRight");
//                    printf("Pivot Right\n\r");
                    break;
                case HardRight:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Motor_Right(FULL_MOTOR/active, 0);
                    Clock_Delay1ms(TURN_90_TIME);
                    Nokia5110_OutString("HardRight");
//                    printf("Hard Right\n\r");
                    break;
                case LC:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Motor_Forward(FULL_MOTOR/active, FULL_MOTOR/active);
                    Nokia5110_OutString("LC");
//                    printf("LC\n\r");
                    Clock_Delay1ms(TURN_90_TIME);
                    break;
                case HardLeft:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    Motor_Left(0, FULL_MOTOR/active);
                    Nokia5110_OutString("HardLeft");
//                    printf("Hard Left\n\r");
                    Clock_Delay1ms(TURN_90_TIME);
                    break;
                case RC:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Motor_Forward(FULL_MOTOR/active, FULL_MOTOR/active);
                    Nokia5110_OutString("RC");
//                    printf("RC\n\r");
                    Clock_Delay1ms(TURN_90_TIME);
                    break;
                case Lost:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Motor_Backward(HALF_MOTOR/active, HALF_MOTOR/active);
                    Nokia5110_OutString("Lost");
//                    printf("Lost\n\r");
                    break;
            }
    }
}
