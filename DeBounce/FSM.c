# include "FSM.h"

#define FALSE 0
#define TRUE 1

void InitializeFSM(FSMType *FSM)
{
    FSM->CurrentState = BothLEDsOff;
}

FSMState NextStateFunction(FSMType *FSM, DebounceSwitchFSMDefine *DebounceSwitchFSM)
{
    FSMState NextState = FSM->CurrentState;
    uint8_t DebounceSwitchFSMCycleNotComplete = DebounceSwitchFSM->SwitchCycleNotComplete;

    switch (FSM->CurrentState) {
    case BothLEDsOff:
        if (DebounceSwitchFSMCycleNotComplete == FALSE) {
            NextState = RedOffGreenOn;
        }
        else {
            NextState = BothLEDsOff;
        }
        break;
    case RedOffGreenOn:
        if (DebounceSwitchFSMCycleNotComplete == FALSE) {
            NextState = RedOnGreenOff;
        }
        else {
            NextState = RedOffGreenOn;
        }
        break;
    case RedOnGreenOff:
        if (DebounceSwitchFSMCycleNotComplete == FALSE) {
            NextState = BothLEDsOn;
        }
        else {
            NextState = RedOnGreenOff;
        }
        break;
    case BothLEDsOn:
        if (DebounceSwitchFSMCycleNotComplete == FALSE) {
            NextState = BothLEDsOff;
        }
        else {
            NextState = BothLEDsOn;
        }
        break;
    }

    return NextState;
}

void OutputFunction(FSMType *FSM, DebounceSwitchFSMDefine *DebounceSwitchFSM)
{
    if (DebounceSwitchFSM->SwitchCycleNotComplete == FALSE) {
        DebounceSwitchFSM->SwitchCycleNotComplete = TRUE;
    }

    switch (FSM->CurrentState) {
    case BothLEDsOff:
        TURN_OFF_LED2_B;
        TURN_OFF_LED1;
        break;
    case RedOffGreenOn:
        TURN_ON_LED2_B;
        TURN_OFF_LED1;
        break;
    case RedOnGreenOff:
        TURN_OFF_LED2_B;
        TURN_ON_LED1;
        break;
    case BothLEDsOn:
        TURN_ON_LED2_B;
        TURN_ON_LED1;
        break;
    }
}
