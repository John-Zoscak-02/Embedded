#ifndef FSM_H_
#define FSM_H_

#include "PortPins.h"
#include "Switch.h"

// Type Definitions
typedef enum {
    LEDHeartBeatStart,
    LEDHeartBeat,
    LEDWaitingToBeat,
    LEDPauseHeartBeat,
    LEDColorCyclingStart,
    LEDSignalsWait,
    LEDSignals,
    LEDColorCycling,
    LEDWaitingToCycle,
    LEDPauseCycle,
    LEDStop
} FSMState;

typedef struct {
    FSMState     CurrentState;      // Current state of the FSM
    SwitchStatus CurrentInputS1;    // Current input of the FSM S1
    SwitchStatus CurrentInputS2;    // Current input of the FSM S2
} FSMType;

// Function Prototypes
void InitializeFSM(FSMType *FSM);
FSMState NextStateFunction(FSMType *FSM);
void OutputFunction(FSMType *FSM);

#endif /* FSM_H_ */
