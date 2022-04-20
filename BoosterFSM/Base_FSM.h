#ifndef BASE_FSM_H_
#define BASE_FSM_H_

#include <stdint.h>

// Type Definitions
typedef enum {
    LightOut,
    Inactive,
    Normal,
    Slow,
    Bumped,
    GoingUp,
    StopUp,
    PickedUp,
    GoingDown,
    StoppedDown,
} BaseFSMState;

typedef struct {
    BaseFSMState    CurrentState;       // Current state of the FSM
    uint8_t         BumpInput;          // Active bumps according to Bump_Input()
    uint8_t         LaunchPadInput;     // Input from the pushbuttons
    float           LightLevel;         // Input from the light sensor
    double          AccelZ;
} BaseFSMType;

// Function Prototypes
void InitializeBaseFSM(BaseFSMType *FSM);
BaseFSMState BaseNextStateFunction(BaseFSMType *FSM);
uint8_t BaseOutputFunction(BaseFSMType *FSM);

#endif /* LINE_FSM_H_ */
