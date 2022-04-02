#include "FSM.h"
#include "Clock.h"

#define CYCLING_DELAY_MS     500
#define HEARTBEAT_DELAY_MS   100
#define DEBOUNCING_DELAY_MS  100

//--------------------------------------------------------------------------
// Initialize FSM
//--------------------------------------------------------------------------
void InitializeFSM(FSMType *FSM)
{
    FSM->CurrentState = LEDHeartBeatStart;
}

//--------------------------------------------------------------------------
// Determine next FSM state
//--------------------------------------------------------------------------
FSMState NextStateFunction(FSMType *FSM)
{
    FSMState NextState = FSM->CurrentState;

    switch (FSM->CurrentState){
//--Heart Beating states----------------------------------------------------
    case LEDHeartBeatStart:
        if(FSM->CurrentInputS2 == Active){
            NextState = LEDPauseHeartBeat;
        }
        else if(FSM->CurrentInputS1 == Active){
            NextState = LEDHeartBeat;
        }
        else {
            NextState = LEDHeartBeatStart;
        }
        break;
    case LEDHeartBeat:
        //Ensures S2 has precedence
        if(FSM->CurrentInputS2 == Active){
            NextState = LEDPauseHeartBeat;
        }
        else if(FSM->CurrentInputS1 == Active){
            NextState = LEDHeartBeat;
        }
        else {
            NextState = LEDColorCyclingStart;
        }
        break;
//--Heart Beat paused states------------------------------------------------
    case LEDPauseHeartBeat:
        //Only S2 can change this state
        if(FSM->CurrentInputS2 == Inactive){
            NextState = LEDWaitingToBeat;
        }
        else {
            NextState = LEDPauseHeartBeat;
        }
        break;
    case LEDWaitingToBeat:
        //A delay is a simple way ignore noisy switch signals
        //Typically, you don't want a lot of delays in code
        //because you might miss something important (like bump sensor readings)
        //later in the course you'll be introduced to better debouncing methods
        Clock_Delay1ms(DEBOUNCING_DELAY_MS);
        //Only S2 can change its state
        if(FSM->CurrentInputS2 == Inactive){
            NextState = LEDWaitingToBeat;
        }
        else {
            NextState = LEDHeartBeatStart;
        }
        break;
//--Color cycling states----------------------------------------------------
    case LEDColorCyclingStart:
        //Ensures S2 has precedence
        if(FSM->CurrentInputS2 == Active){
            NextState = LEDPauseCycle;
        }
        else if(FSM->CurrentInputS1 == Active){
            NextState = LEDColorCycling;
        }
        else {
            NextState = LEDColorCyclingStart;
        }
        break;

    case LEDSignalsWait:
        if (FSM->CurrentInputS1 == Inactive) {
            NextState = LEDSignals;
        }
        else {
            NextState = LEDSignalsWait;
        }
        break;
    case LEDSignals:
        if (FSM->CurrentInputS1 == Active) {
            NextState = LEDColorCycling;
        }
        else {
            NextState = LEDSignals;
        }
    case LEDColorCycling:
        //Ensures S2 has precedence
        if(FSM->CurrentInputS2 == Active){
            NextState = LEDPauseCycle;
        }
        else if(FSM->CurrentInputS1 == Active){
            NextState = LEDColorCycling;
        }
        else {
            NextState = LEDStop;
        }
        break;
//--Color cycling paused state----------------------------------------------
    case LEDPauseCycle:
        //Only S2 can change its state
        if(FSM->CurrentInputS2 == Inactive){
           NextState = LEDWaitingToCycle;
        }
        else {
           NextState = LEDPauseCycle;
        }
        break;
    case LEDWaitingToCycle:
        //See note in LEDWaitingToBeat regarding delay
        Clock_Delay1ms(DEBOUNCING_DELAY_MS);
        //Only S2 can change its state
        if(FSM->CurrentInputS2 == Inactive){
           NextState = LEDWaitingToCycle;
        }
        else{
           NextState = LEDColorCyclingStart;
        }
        break;
//--Stop LED----------------------------------------------------------------
    case LEDStop:
        if(FSM->CurrentInputS1 == Active){
            NextState = LEDHeartBeatStart;
        }
        else {
            NextState = LEDStop;
        }
        break;
    }
    return NextState;
}

//--------------------------------------------------------------------------
// Determine LED output based on state
//--------------------------------------------------------------------------
void OutputFunction(FSMType *FSM)
{
    switch (FSM->CurrentState) {
    case LEDHeartBeatStart:
        TURN_OFF_LED2_BLUE;
        TURN_ON_LED2_RED;
        Clock_Delay1ms(HEARTBEAT_DELAY_MS);
        TURN_OFF_LED2_RED;
        Clock_Delay1ms(HEARTBEAT_DELAY_MS);
        break;
    case LEDHeartBeat:
        // Do nothing (release happens too quickly to see behavior change)
        break;
    case LEDPauseHeartBeat:
        TURN_ON_LED2_RED;
        break;
    case LEDWaitingToBeat:
        // Do nothing (release happens too quickly to see behavior change)
        break;
    case LEDColorCyclingStart:
        TURN_ON_LED2_BLUE;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        TURN_OFF_LED2_BLUE;
        TURN_ON_LED2_RED;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        TURN_ON_LED2_BLUE;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        TURN_OFF_LED2_RED;
        TURN_OFF_LED2_BLUE;
        TURN_ON_LED2_GREEN;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        TURN_ON_LED2_BLUE;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        TURN_OFF_LED2_BLUE;
        TURN_ON_LED2_RED;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        TURN_ON_LED2_BLUE;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        TURN_OFF_LED2_RED;
        TURN_OFF_LED2_BLUE;
        TURN_OFF_LED2_GREEN;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        break;
    case LEDSignalsWait:
        break;
    case LEDSignals:
        TURN_ON_LED8_FL;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        TURN_OFF_LED8_FL;
        TURN_ON_LED8_FR;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        TURN_OFF_LED8_FR;
        TURN_ON_LED8_BR;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        TURN_OFF_LED8_BR;
        TURN_ON_LED8_BL;
        Clock_Delay1ms(CYCLING_DELAY_MS);
        TURN_OFF_LED8_BL;
        break;
    case LEDColorCycling:
        // Do nothing (release happens too quickly to see behavior change)
        break;
    case LEDPauseCycle:
        TURN_ON_LED2_BLUE;
        break;
    case LEDWaitingToCycle:
        // Do nothing (release happens too quickly to see behavior change)
        break;
    case LEDStop:
        TURN_OFF_LED2_BLUE;
    }

}