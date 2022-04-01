#ifndef __TIMERA0INTS_H__ // do not include more than once
#define __TIMERA0INTS_H__

#include "msp.h"
#include <stdint.h>

#include "PortPins.h"

// Function prototypes
void InitializeTimerA0(void(*task)(void), uint16_t CCR0Value);
void StopTimerA0(void);

#endif
