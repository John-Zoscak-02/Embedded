#ifndef FSM_H_
#define FSM_H_

#include <stdint.h>

// Type Definitions
typedef enum {
    Inactive,
    PivotLeft,
    CheckPivotLeft,
    RotateLeft,
    CheckRotateLeft,
    ForwardLeft,
    CheckForwardLeft,
    Center,
    CheckCenter,
    ForwardRight,
    CheckForwardRight,
    RotateRight,
    CheckRotateRight,
    PivotRight,
    CheckPivotRight,
    Bumped,
    Lost,
} FSMState;

typedef struct {
    FSMState    CurrentState;           // Current state of the FSM
//    FSMState    PreviousState;          // The previous state
    uint8_t     Position;               // Current Position according to Reflectance_Center()
    uint8_t     Lost;
    uint8_t     BumpInput;              // Active bumps according to Bump_Input()
    uint8_t     LaunchPadInput;         // Input from the pushbuttons
} FSMType;

// Function Prototypes
void InitializeFSM(FSMType *FSM);
FSMState NextStateFunction(FSMType *FSM);
void OutputFunction(FSMType *FSM);

#endif /* FSM_H_ */