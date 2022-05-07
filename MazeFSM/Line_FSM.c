#include "Line_FSM.h"
#include "driverlib.h"
#include "Reflectance.h"
#include "Motor.h"
#include "Clock.h"
#include "Nokia5110.h"
#include "UART0.h"

/////// Maze speed
#define FULL_MOTOR 20
#define HALF_MOTOR 12

#define TURN_90_TIME 725

/////// Line follower speed
//#define FULL_MOTOR 40
//#define HALF_MOTOR 15

//#define TURN_90_TIME 450

#define DENSITY_TO_TURN_90 4
#define DENSITY_OF_T 6

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
            if (FSM->Density < DENSITY_TO_TURN_90) {
                if (FSM->Lost) {
                    NextState = HardRight;
                }
                else if (-334 <= FSM->Position && FSM->Position <= -239) {
                    NextState = PivotLeft;
                }
                else if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else {
                    NextState = Check;
                }
            }
            else {
                if (FSM->MazeEnd) {
                    NextState = CheckMazeEnd;
                }
                else if (FSM->Position >= -48 || FSM->Density >= DENSITY_OF_T) {
                    NextState = TRight;
                }
                else {
                    NextState = CheckTLeft;
                }
            }
            break;
        case RotateLeft:
            if (FSM->Density < DENSITY_TO_TURN_90) {
                if (FSM->Lost) {
                    NextState = HardRight;
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
                    NextState = Check;
                }
            }
            else {
                if (FSM->MazeEnd) {
                    NextState = CheckMazeEnd;
                }
                else if (FSM->Position >= -48 || FSM->Density >= DENSITY_OF_T) {
                    NextState = TRight;
                }
                else {
                    NextState = CheckTLeft;
                }
            }
            break;
        case ForwardLeft:
            if (FSM->Density < DENSITY_TO_TURN_90) {
                if (-238 <= FSM->Position && FSM->Position <= -143) {
                    NextState = RotateLeft;
                }
                else if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48 && !FSM->Lost) {
                    NextState = Center;
                }
                else {
                    NextState = Check;
                }
            }
            else {
                if (FSM->MazeEnd) {
                    NextState = CheckMazeEnd;
                }
                else if (FSM->Position >= -48  || FSM->Density >= DENSITY_OF_T) {
                    NextState = TRight;
                }
                else {
                    NextState = CheckTLeft;
                }
            }
            break;
        case Center:
            // Non-check motor states can only go directly to adjacent motor-states
            if (FSM->Density < DENSITY_TO_TURN_90) {
                if (-142 <= FSM->Position && FSM->Position <= -48) {
                    NextState = ForwardLeft;
                }
                else if (-48 <= FSM->Position && FSM->Position <= 48 && !FSM->Lost) {
                    NextState = Center;
                }
                else if (48 <= FSM->Position && FSM->Position <= 142) {
                    NextState = ForwardRight;
                }
                else {
                    NextState = Check;
                }
            }
            else {
                if (FSM->MazeEnd) {
                    NextState = CheckMazeEnd;
                }
                else if (FSM->Position >= -48  || FSM->Density >= DENSITY_OF_T) {
                    NextState = TRight;
                }
                else {
                    NextState = CheckTLeft;
                }
            }
            break;
        case Check:
            if (FSM->Density < DENSITY_TO_TURN_90) {
                if (FSM->Lost) {
//                    NextState = Lost;
                    NextState = TurnAround;
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
            else {
                if (FSM->MazeEnd) {
                    NextState = CheckMazeEnd;
                }
                else if (FSM->Position >= -48  || FSM->Density >= DENSITY_OF_T) {
                    NextState = TRight;
                }
                else {
                    NextState = CheckTLeft;
                }
            }
            break;
        case ForwardRight:
            if (FSM->Density < DENSITY_TO_TURN_90) {
                if (-48 <= FSM->Position && FSM->Position <= 48 && !FSM->Lost) {
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
            }
            else {
                if (FSM->MazeEnd) {
                    NextState = CheckMazeEnd;
                }
                else if (FSM->Position >= -48  || FSM->Density >= DENSITY_OF_T) {
                    NextState = TRight;
                }
                else {
                    NextState = CheckTLeft;
                }
            }
            break;
        case RotateRight:
            if (FSM->Density < DENSITY_TO_TURN_90) {
                // Non-check motor states can only go directly to adjacent motor-states
                if (FSM->Lost) {
                    NextState = HardLeft;
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
                if (FSM->MazeEnd) {
                    NextState = CheckMazeEnd;
                }
                else if (FSM->Position >= -48 || FSM->Density >= DENSITY_OF_T) {
                    NextState = TRight;
                }
                else {
                    NextState = CheckTLeft;
                }
            }
            break;
        case PivotRight:
            if (FSM->Density < DENSITY_TO_TURN_90) {
                // Non-check motor states can only go directly to adjacent motor-states
                if (FSM->Lost) {
                    NextState = HardLeft;
                }
                else if (143 <= FSM->Position && FSM->Position <= 238) {
                    NextState = RotateRight;
                }
                else if (239 <= FSM->Position && FSM->Position <= 334) {
                    NextState = PivotRight;
                }
            }
            else {
                if (FSM->MazeEnd) {
                    NextState = CheckMazeEnd;
                }
                else if (FSM->Position >= -48  || FSM->Density >= DENSITY_OF_T) {
                    NextState = TRight;
                }
                else {
                    NextState = CheckTLeft;
                }
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
        case CheckTLeft:
            if (FSM->Lost) {
                NextState = TLeft;
            }
            else {
                NextState = Center;
            }
            break;
        case TLeft:
            NextState = Center;
            break;
        case TRight:
            NextState = Center;
            break;
        case TurnAround:
            NextState = Center;
            break;
        case CheckMazeEnd:
            if (FSM->MazeEnd) {
                NextState = MazeEnd;
            }
            else {
                NextState = CheckMazeEnd;
            }
        case MazeEnd:
            NextState = MazeEnd;
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
            switch (FSM->CurrentState) {
                // If in a left turning motor state, turn on the left turn signal and turn off the right turn signal
                case PivotLeft:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Pivot left
                    Motor_Left(FULL_MOTOR/active, FULL_MOTOR/active);
                    Nokia5110_OutString("PivotLeft");
//                    printf("Pivot Left\n\r");
                    break;
                case RotateLeft:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Slight left pivot
                    Motor_Left(0, FULL_MOTOR/active);
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
                case Check:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Nokia5110_OutString("Check");
//                    printf("Check\n\r");
                    break;
                // If in a right moving state, turn on the right turn signal
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
                    Motor_Right(FULL_MOTOR/active, 0);
                    Nokia5110_OutString("RotateRight");
//                    printf("Rotate Right\n\r");
                    break;
                case PivotRight:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    // Pivot Right
                    Motor_Right(FULL_MOTOR/active, FULL_MOTOR/active);
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
                case CheckTLeft:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Nokia5110_OutString("CheckTLeft");
                    // Check to see if there is a forward path and take it as opposed to the left turn that was just detected
                    Motor_Forward(FULL_MOTOR/active, FULL_MOTOR/active);
                    Clock_Delay1ms(TURN_90_TIME/6);
                    break;
                case TLeft:
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Nokia5110_OutString("TLeft");
                    // Just came from TLeft, move back to where the turn actually is
//                    Motor_Backward(FULL_MOTOR/active, FULL_MOTOR/active);
//                    Clock_Delay1ms(TURN_90_TIME/4);
                    // Take the left turn
                    Motor_Left(FULL_MOTOR/active, FULL_MOTOR/active);
                    Clock_Delay1ms(TURN_90_TIME);
                    // Move forward a little to not get conflicting input from the reflectance as it pivots over the turn
                    Motor_Forward(FULL_MOTOR/active, FULL_MOTOR/active);
                    Clock_Delay1ms(TURN_90_TIME/4);
                    break;
                case TRight:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Nokia5110_OutString("TRight");
                    // Move the robot forward so that the center of rotation is exactly above the turn
                    Motor_Forward(FULL_MOTOR/active, FULL_MOTOR/active);
                    Clock_Delay1ms(TURN_90_TIME/8);
                    // Take the turn
                    Motor_Right(FULL_MOTOR/active, FULL_MOTOR/active);
                    Clock_Delay1ms(TURN_90_TIME);
                    // Move forward a little to not get conflicting input from the reflectance as it pivots over the turn
                    Motor_Forward(FULL_MOTOR/active, FULL_MOTOR/active);
                    Clock_Delay1ms(TURN_90_TIME/4);
                    break;
                case TurnAround:
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Nokia5110_OutString("TurnAround");
                    Motor_Right(FULL_MOTOR/active, FULL_MOTOR/active);
                    Clock_Delay1ms(TURN_90_TIME*2);
                    Motor_Forward(FULL_MOTOR/active, FULL_MOTOR/active);
                    break;
                case CheckMazeEnd:
                    Clock_Delay1ms(TURN_90_TIME/6);
                    break;
                case MazeEnd:
                    Nokia5110_OutString("MazeEnd");
                    Motor_Stop();
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Clock_Delay1ms(1000);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
                    Clock_Delay1ms(1000);
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
