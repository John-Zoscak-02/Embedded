#include "LED.h"

void InitializeLEDPortPins(void)
{
	// Initialize the value and port pin direction of the LEDs.
    TURN_OFF_LED1;
    SET_LED1_AS_AN_OUTPUT;

    TURN_OFF_LED2_R;
    SET_LED2_R_AS_AN_OUTPUT;
    TURN_OFF_LED2_G;
    SET_LED2_G_AS_AN_OUTPUT;
    TURN_OFF_LED2_B;
    SET_LED2_B_AS_AN_OUTPUT;
}

