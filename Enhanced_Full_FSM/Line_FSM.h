#ifndef LINE_FSM_H_
#define LINE_FSM_H_

#include <stdint.h>

// Type Definitions
typedef enum {
    PivotLeft,
    RotateLeft,
    ForwardLeft,
    AdjustL,
    Center,
    AdjustR,
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
    int16_t         Position;               // Current Position according to Reflectance_Center()
    uint16_t         Density;                // The number of reflectance bits that are 1
    uint8_t         MazeEnd;                // If the maze has a special input then it knows that it solved it
    uint8_t         Lost;                   // If the reflectance is not picking up anything
    uint8_t         cross;
} LineFSMType;

// Function Prototypes
void InitializeLineFSM(LineFSMType *FSM);
LineFSMState LineNextStateFunction(LineFSMType *FSM, uint8_t active);
void LineOutputFunction(LineFSMType *FSM, uint8_t active);

#endif /* LINE_FSM_H_ */
