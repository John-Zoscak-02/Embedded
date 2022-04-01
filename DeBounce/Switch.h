#ifndef SWITCH_H_
#define SWITCH_H_

#include "msp.h"
#include <stdint.h>

typedef enum {
    Inactive, Active
} SwitchStatus;

typedef struct {
    uint8_t * SwitchPort;                   // Input port associated with switch
    uint8_t SwitchPortBit;                  // Bit mask for port pin associated with switch
    SwitchStatus PortPinIs0Value;               // Switch state associated with value 0.
    SwitchStatus PortPinIs1Value;               // Switch state associated with value 1.
} SwitchDefine;

// Function Prototypes
void InitializeSwitch(SwitchDefine *Switch,uint8_t *SwitchPort,uint8_t SwitchBit,
                      SwitchStatus PortPinIs0Value, SwitchStatus PortPinIs1Value);

//This function returns the instantaneous value of the selected switch
SwitchStatus ReadSwitchStatus(SwitchDefine *Switch);

#endif /* SWITCH_H_ */
