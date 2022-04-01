/*
 * InterruptsMain.c
 */
#include "Bump.h"
#include "Clock.h"
#include "UART0.h"
#include "LaunchPad.h"
#include <stdint.h>

void Hello_World_Example(void){
    Clock_Init();
    Bump_Init();
    UART0_Init();
    while(1){
        UART0_OutString("Hello Wolrd /n/r");
        Clock_Delay1ms(1000);
    }
}

void Interrupts_Warm_Up(void){
    Clock_Init();
    Bump_Init();
    UART0_Init();
    while(1){
        //Write code to print which bump sensors are pressed using UART
         //uint8_t read = FSM->BumpInput;
        // if there has been a bump input, print to UART
        if (read) {
            // deconstruct the input, print a 0 for a negative pin input and 1 for a positive pin input
            UART0_OutString(0x20 & read ? "1" : "0");
            UART0_OutString(0x10 & read ? "1" : "0");
            UART0_OutString(0x08 & read ? "1" : "0");
            UART0_OutString(0x04 & read ? "1" : "0");
            UART0_OutString(0x02 & read ? "1" : "0");
            UART0_OutString(0x01 & read ? "1" : "0");
            // newline
            UART0_OutString("\n\r");
        }
        //Wait a second before polling again
        Clock_Delay1ms(1000);
    }
}

void toggleLedFor2Seconds(uint8_t bumpSensors){
    // Print to UART that the bump task has been called
    UART0_OutString("Bump task called");
    UART0_OutString("\n\r");
    LaunchPad_LED(1);

    // deconstruct the input, print a 0 for a negative pin input and 1 for a positive pin input
    UART0_OutString(0x20 & bumpSensors ? "1" : "0");
    UART0_OutString(0x10 & bumpSensors ? "1" : "0");
    UART0_OutString(0x08 & bumpSensors ? "1" : "0");
    UART0_OutString(0x04 & bumpSensors ? "1" : "0");
    UART0_OutString(0x02 & bumpSensors ? "1" : "0");
    UART0_OutString(0x01 & bumpSensors ? "1" : "0");
    // newline
    UART0_OutString("\n\r");

    Clock_Delay1ms(2000);
    LaunchPad_LED(0);
    UART0_OutString("Bump task returns");
    UART0_OutString("\n\r");
}

void Interrupts_Race_Day(void){
    //Initializations
    Clock_Init();
    UART0_Init();
    Bump_Interrupt_Init(&toggleLedFor2Seconds);
    LaunchPad_Init();
    while(1){
    }
}

int main(void){
    //Un-commment only the function you are using
//    Hello_World_Example();
//    Interrupts_Warm_Up();
    Interrupts_Race_Day();
}
