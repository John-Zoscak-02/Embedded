#ifndef LINE_FSM_H_
#define LINE_FSM_H_

#include <stdint.h>

// Type Definitions
typedef enum {
    PivotLeft,
    RotateLeft,
    ForwardLeft,
    Center,
    Check,
    ForwardRight,
    RotateRight,
    PivotRight,
    HardRight,
    LC,
    HardLeft,
    RC,
    Lost,
} LineFSMState;

typedef struct {
    LineFSMState    CurrentState;           // Current state of the FSM
    int16_t     Position;               // Current Position according to Reflectance_Center()
    uint8_t     Lost;
} LineFSMType;

// Function Prototypes
void InitializeLineFSM(LineFSMType *FSM);
LineFSMState LineNextStateFunction(LineFSMType *FSM, uint8_t active);
void LineOutputFunction(LineFSMType *FSM, uint8_t active);

#endif /* LINE_FSM_H_ */
