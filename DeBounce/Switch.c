#include "Switch.h"

SwitchStatus ReadSwitchStatus(SwitchDefine *Switch)
{
    volatile uint8_t SwitchValue;

    SwitchValue = *(Switch->SwitchPort) & Switch->SwitchPortBit;
    if (SwitchValue)
        return Switch->PortPinIs1Value;
    else
        return Switch->PortPinIs0Value;
}

void InitializeSwitch(SwitchDefine *Switch,uint8_t *SwitchPort,uint8_t SwitchBit,
        SwitchStatus PortPinIs0Value, SwitchStatus PortPinIs1Value)
{
    Switch->SwitchPort = SwitchPort;
    Switch->SwitchPortBit = SwitchBit;
    Switch->PortPinIs0Value= PortPinIs0Value;
    Switch->PortPinIs1Value = PortPinIs1Value;
}
