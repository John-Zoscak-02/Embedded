/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "Clock.h"
#include "TimerA1.h"
#include "UART0.h"
#include "LaunchPad.h"

volatile uint32_t counter;

void Task(void) {
    counter++;
    UART0_OutString("in task: ");
    UART0_OutUDec(counter);
    UART0_OutString("\n\r");
}

int main(void) {
    // Init the clock
    Clock_Init();
    // Initialize the Timer A1 to interrupt in a period of 3200 ticks, where clock is going at 500mHz
    TimerA1_Init(&Task, 32000);
    //Initialize UART for demonstration that the task is executed regularly
    UART0_Init();
    // Initialize the counter to show sequential calls of the task regularly as the interrupt is activated.
    counter = 0;
    // Main loop
    while(1){
    }
}
