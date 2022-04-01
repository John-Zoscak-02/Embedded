#ifndef FSM_H_
#define FSM_H_

#include <stdint.h>

// Type Definitions
typedef enum {
    Inactive,
    Bumped,
    PosLeft,
    PosRight,
    PosCenter,
    PosLost
} FSMState;

typedef struct {
    FSMState    CurrentState;           // Current state of the FSM
    uint8_t     Position;               // Current Position according to Reflectance_Center()
    uint8_t     BumpInput;              // Active bumps according to Bump_Input()
} FSMType;

// Function Prototypes
void InitializeFSM(FSMType *FSM);
FSMState NextStateFunction(FSMType *FSM);
void OutputFunction(FSMType *FSM);

#endif /* FSM_H_ */
