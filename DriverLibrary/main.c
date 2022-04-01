#include "msp.h"
#include "driverlib.h"

#define DELAY 500
int i;

// "empty" project so these can be seen within CCS
void main(void)
{
    WDT_A_holdTimer();

    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1|GPIO_PIN4);

    while (1) {
        if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1)==GPIO_INPUT_PIN_LOW) {
            for(i = 0;i<DELAY;i++);
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == GPIO_INPUT_PIN_LOW) {
                GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);
                while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == GPIO_INPUT_PIN_LOW);
            }
        }
        if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4)==GPIO_INPUT_PIN_LOW) {
            for(i = 0;i<DELAY;i++);
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == GPIO_INPUT_PIN_LOW) {
                GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
                while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == GPIO_INPUT_PIN_LOW);
            }
        }
    }
}
