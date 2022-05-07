/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* Project Includes */
#include "Clock.h"
#include "msp.h"
#include "TimerA1.h"
#include "UART0.h"
#include "LaunchPad.h"
#include "Reflectance.h"
#include "Bump.h"
#include "PWM.h"
#include "Line_FSM.h"
#include "Base_FSM.h"
#include "Nokia5110.h"
#include "Motor.h"
#include "CortexM.h"
#include "Booster_Pack/opt3001.h"
#include "Booster_Pack/i2c_driver.h"
#include "Booster_Pack/bmi160_support.h"
#include "Booster_Pack/bme280_support.h"
#include <math.h>

volatile uint8_t counter;
volatile uint8_t reflectance_input;
volatile uint8_t bump_input;
volatile uint8_t launchpad;

struct bmi160_accel_t s_accelXYZ;
int8_t accel_off_x;
int8_t accel_off_y;
int8_t accel_off_z;
BMI160_RETURN_FUNCTION_TYPE returnValue;
int returnRslt;

#define USING_BOSCH_BP
#define SAMPLE_TIME_1               (53)
#define SAMPLE_TIME_2               (26)
#define SAMPLE_TIME_4               (13)
#define SAMPLE_TIME_6               (8)
#define SAMPLE_TIME_8               (6)
#define SAMPLE_TIME_10              (5)

void startCrystalOscillator(void);
void setSystemClock(uint32_t CPU_Frequency);
void configureGPIO(void);
void startWakeUpTimerA(uint16_t ulClockMS);
void stopWakeUpTimerA(void);
int32_t movingAvg(int prevAvg, int16_t newValue);

volatile uint8_t wdtWakeUpPeriodIndex = 6;

const uint8_t wdtWakeUpPeriod [8] = {
        WDT_A_CLOCKDIVIDER_2G,
        WDT_A_CLOCKDIVIDER_128M,
        WDT_A_CLOCKDIVIDER_8192K,
        WDT_A_CLOCKDIVIDER_512K,
        WDT_A_CLOCKDIVIDER_32K,
        WDT_A_CLOCKDIVIDER_8192,
        WDT_A_CLOCKDIVIDER_512,
        WDT_A_CLOCKDIVIDER_64,
};

const uint8_t timeSamplesBMI [6] = {
        SAMPLE_TIME_1,      //Sample at 1 time per second
        SAMPLE_TIME_2,      //Sample at 2 times per second
        SAMPLE_TIME_4,      //Sample at 4 times per second
        SAMPLE_TIME_6,      //Sample at 6 times per second
        SAMPLE_TIME_8,      //Sample at 8 times per second
        SAMPLE_TIME_10,     //Sample at 10 times per second
};

const uint8_t timeSamplesBMM [6] = {
        SAMPLE_TIME_1,      //Sample at 1 time per second
        SAMPLE_TIME_2,      //Sample at 2 times per second
        SAMPLE_TIME_4,      //Sample at 4 times per second
        SAMPLE_TIME_6,      //Sample at 6 times per second
        SAMPLE_TIME_8,      //Sample at 8 times per second
        SAMPLE_TIME_10,     //Sample at 10 times per second
};

const uint8_t timeSamplesBME [6] = {
        SAMPLE_TIME_1,      //Sample at 1 time per second
        SAMPLE_TIME_2,      //Sample at 2 times per second
        SAMPLE_TIME_4,      //Sample at 4 times per second
        SAMPLE_TIME_6,      //Sample at 6 times per second
        SAMPLE_TIME_8,      //Sample at 8 times per second
        SAMPLE_TIME_10,     //Sample at 10 times per second
};

const uint8_t timeSamplesTMP [6] = {
        SAMPLE_TIME_1,      //Sample at 1 time per second
        SAMPLE_TIME_2,      //Sample at 2 times per second
        SAMPLE_TIME_4,      //Sample at 4 times per second
        SAMPLE_TIME_6,      //Sample at 6 times per second
        SAMPLE_TIME_8,      //Sample at 8 times per second
        SAMPLE_TIME_10,     //Sample at 10 times per second
};

const uint8_t timeSamplesOPT [6] = {
        SAMPLE_TIME_1,      //Sample at 1 time per second
        SAMPLE_TIME_2,      //Sample at 2 times per second
        SAMPLE_TIME_4,      //Sample at 4 times per second
        SAMPLE_TIME_6,      //Sample at 6 times per second
        SAMPLE_TIME_8,      //Sample at 8 times per second
        SAMPLE_TIME_10,     //Sample at 10 times per second
};

// Task to be called on a periodic basis by timer a1
void Task() {
//    UART0_OutString("In Task \n\r");
    counter++;
    // on every tenth call of task, call reflectance start
    if (counter == 10) {
        Reflectance_Start();
    }
    // on every tenth call of task (delayed by 1), call reflectance end and the position global variable to it
    else if (counter == 11) {
        reflectance_input = Reflectance_End();
    }
    // on every tenth call of task (delayed by 2), call bump read and set the bump read global variable to it
    // reset counter to two, this will ensure that ten cycles passed between the recent reflectance start and the next one
    else if (counter >= 12) {
        counter = 8;
        bump_input = Bump_Read();
    }
}

int16_t position(uint8_t data) {
   signed short WEIGHTS[] = {334, 238, 142, 48, -48, -142, -238, -334};
   // Declare kbits to count the number of bits that were 1
   uint8_t kbits = 0;
   // Declare kpos for arithmetic to determine the displacement from the center
   int32_t kpos = 0;
   uint8_t i = 0;
   for (i = 0; i < 8; i++) {
       // If a pin was 1, then add it to kpos and increment the kbits counter
       if (data & 0x01) {
           kpos += WEIGHTS[i];
           kbits++;
       }
       // Look at the next pin in the next loop
       data = data >> 1;
   }
   // return average of all of the weights of the active pins
   if (kbits <= 2) {
       return kpos / kbits;
   }
   else {
       return kpos;
   }
}

// Calculates the number of true outputs from the reflectance input
uint16_t density(uint8_t data) {
    uint16_t density = 0;
    int i = 0;
    for (i = 0; i < 8; i++) {
           // If a pin was 1, then add it to kpos and increment the kbits counter
           if (data & 0x01) {
               density++;
           }
           // Look at the next pin in the next loop
           data = data >> 1;
       }
    return density;
}

// Calculates the number of true outputs from the reflectance input
// Booster Sensors have been disabled for sanity purposes
// Problems with I2C are pretty unfixable
 int main(void) {
     DisableInterrupts();
     Clock_Init();
//     UART0_Initprintf(); //so we can use the cool printf
     EnableInterrupts();
     //intialize i2c to communicate with booster pack
     initI2C();

    PWM_Init(1, 0, 0);
    Motor_Init();
    TimerA1_Init(&Task, 500);
    Nokia5110_Init();

    //Intialize opt3001 sensor
    sensorOpt3001Init();
    sensorOpt3001Enable(true);

    //Intialize BMI160 and BMM sensor
    bmi160_initialize_sensor();
    returnRslt = bmi160_config_running_mode(APPLICATION_NAVIGATION); //there are several modes to run the BMI sensor in
    bmi160_accel_foc_trigger_xyz(0x03, 0x03, 0x01, &accel_off_x, &accel_off_y, &accel_off_z);

    LineFSMType Line_Center_FSM;
    BaseFSMType Base_FSM;

    // Counter, the global position variable inits to zero, the global bump variable inits to zero
    counter = 0;
    reflectance_input = 0x00;
    bump_input = 0x00;
    launchpad = 0x00;
    uint8_t defer_line_fsm = 0b00;

    // Initialize the FSM
    InitializeBaseFSM(&Base_FSM);
    InitializeLineFSM(&Line_Center_FSM);

    while(1){
        // Take Launchpad input directly and pass it to the FSM
        launchpad = LaunchPad_Input();
        Base_FSM.LaunchPadInput = launchpad;

        // Whatever this file's global bump input variable is, give that to the FSMs BumpInput
        Base_FSM.BumpInput = bump_input;

        //reading OPT3001 sensor
        uint16_t rawData;
        float convertedLux;
        //Read and convert OPT values
        sensorOpt3001Read(&rawData);
        sensorOpt3001Convert(rawData, &convertedLux);
        // Pass detected light level to the base FSM
        Base_FSM.LightLevel = convertedLux;
//        Base_FSM.LightLevel = 500;
        // Tell the Base FSM the current Z acceleration
        bmi160_read_accel_xyz(&s_accelXYZ);
        Base_FSM.AccelZ = s_accelXYZ.z;

        // Whatever this file's global position input variable is, give that to the FSMs BumpInput
        Line_Center_FSM.Position = position(reflectance_input);
        // If the reflectance input is all zeros, then we know that
        Line_Center_FSM.Lost = (reflectance_input == 0);

        // Update the Current state of the FSM to the output of the next state that it should be in
        Base_FSM.CurrentState = BaseNextStateFunction(&Base_FSM);
        Line_Center_FSM.CurrentState = LineNextStateFunction(&Line_Center_FSM, defer_line_fsm);

        // Print the density of the reflectance input and the current reflectance input
        Nokia5110_Clear();
        Nokia5110_SetCursor(0, 0);
        Nokia5110_OutString("Pos: ");
//        Nokia5110_SetCursor(0, 4);
//        Nokia5110_OutString("Lum: ");
        Nokia5110_SetCursor(0, 1);
        Nokia5110_OutChar((reflectance_input &0b10000000) ? '1' : '0');
        Nokia5110_SetCursor(1, 1);
        Nokia5110_OutChar((reflectance_input &0b01000000) ? '1' : '0');
        Nokia5110_SetCursor(2, 1);
        Nokia5110_OutChar((reflectance_input &0b00100000) ? '1' : '0');
        Nokia5110_SetCursor(3, 1);
        Nokia5110_OutChar((reflectance_input &0b00010000) ? '1' : '0');
        Nokia5110_SetCursor(4, 1);
        Nokia5110_OutChar((reflectance_input &0b00001000) ? '1' : '0');
        Nokia5110_SetCursor(5, 1);
        Nokia5110_OutChar((reflectance_input &0b00000100) ? '1' : '0');
        Nokia5110_SetCursor(6, 1);
        Nokia5110_OutChar((reflectance_input &0b00000010) ? '1' : '0');
        Nokia5110_SetCursor(7, 1);
        Nokia5110_OutChar((reflectance_input &0b00000001) ? '1' : '0');
//        Nokia5110_SetCursor(7, 4);
//        Nokia5110_OutUDec((uint16_t) convertedLux);
//        Nokia5110_SetCursor(7, 0);
//        Nokia5110_OutSDec(position(reflectance_input));

        // Let the FSM do things based on it's current state
        defer_line_fsm = BaseOutputFunction(&Base_FSM);
        LineOutputFunction(&Line_Center_FSM, defer_line_fsm);

        // Configure WDT
        // For LPM3 Clock Source should be BCLK or VLOCLK
        MAP_WDT_A_initIntervalTimer(WDT_A_CLOCKSOURCE_BCLK/*WDT_A_CLOCKSOURCE_ACLK*/,
                wdtWakeUpPeriod[wdtWakeUpPeriodIndex]);
        MAP_Interrupt_enableInterrupt(INT_WDT_A);

        // Start WDT
        MAP_WDT_A_startTimer();

        //Go to LPM0 (Cannot use LPM3 because we won't accurately receive UART data)
        MAP_PCM_gotoLPM0();
    }
}
 /***********************************************************
   Function: TA0_0_IRQHandler
  */
 void TA0_0_IRQHandler(void)
 {
     MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
             TIMER_A_CAPTURECOMPARE_REGISTER_0);

 #ifdef USE_LPM
     MAP_Interrupt_disableSleepOnIsrExit();
 #endif
 }

 /***********************************************************
   Function: WDT_A_IRQHandler
  */
 void WDT_A_IRQHandler(void)
 {
     //MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
     // Waking up from LMP3 take us to PCM_AM_LDO_VCORE0 instead of PCM_AM_LF_VCORE0
 //  MAP_PCM_setPowerState(PCM_AM_LDO_VCORE0);
 //    MAP_PCM_setCoreVoltageLevel(PCM_AM_DCDC_VCORE0);

 #ifdef USE_LPM
     MAP_Interrupt_disableSleepOnIsrExit();
 #endif
 }

 /***********************************************************
   Function: PORT1_IRQHandler
  */
 void PORT1_IRQHandler(void)
 {
     uint32_t debounce;
     uint32_t status;

     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

     status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);

     if(status & GPIO_PIN1)
     {
     }

     /* Delay for switch debounce */
     for(debounce = 0; debounce < 10000; debounce++)
         MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

 #ifdef USE_LPM
     MAP_Interrupt_disableSleepOnIsrExit();
 #endif
 }

 /***********************************************************
   Function: PORT5_IRQHandler
  */
 void PORT5_IRQHandler(void)
 {
     uint32_t status;

     status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);

     MAP_GPIO_disableInterrupt(GPIO_PORT_P5, GPIO_PIN2);
     MAP_Interrupt_disableInterrupt(INT_PORT5);

     if(status & GPIO_PIN2)
     {
     }

     /* Delay for switch debounce */
     MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

 #ifdef USE_LPM
     MAP_Interrupt_disableSleepOnIsrExit();
 #endif
 }

 /***********************************************************
   Function: _system_pre_init
  */
 int _system_pre_init(void)
 {
     // stop WDT
     MAP_WDT_A_holdTimer();                        // Hold watchdog timer

     // Perform C/C++ global data initialization
     return 1;
 }

