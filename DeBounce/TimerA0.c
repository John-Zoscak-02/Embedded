#include "TimerA0.h"

void (*TimerA0Task)(void);   // user function

// ***************** TimerA0_Init ****************
// Activate Timer A0 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          CCR0Value is the CCR0 value that corresponds to the desired period
// Outputs: none
void InitializeTimerA0(void(*task)(void), uint16_t CCR0Value)
{
    TimerA0Task = task;             // user function

    // Timer A Control Register
    // bits15-10=XXXXXX, reserved
    // bits9-8=10,       clock source to SMCLK
    // bits7-6=10,       input clock divider
    // bits5-4=00,       mode control
    // bit3=X,           reserved
    // bit2=0,           set this bit to clear
    // bit1=0,           no interrupt on timer
    // TIMER_A0->CTL &= ~0x0030;       // halt Timer A0
    //TIMER_A0->CTL = 0x0280;
    TIMER_A0->CTL = (TIMER_A_CTL_MC__STOP | TIMER_A_CTL_CLR);
    TIMER_A0->CTL = (TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__4);

    // Compare/Capture Control Register
    // bits15-14=00,     no capture mode
    // bits13-12=XX,     capture/compare input select
    // bit11=X,          synchronize capture source
    // bit10=X,          synchronized capture/compare input
    // bit9=X,           reserved
    // bit8=0,           compare mode
    // bits7-5=XXX,      output mode
    // bit4=1,           enable capture/compare interrupt on CCIFG
    // bit3=X,           read capture/compare input from here
    // bit2=0,           output this value in output mode 0
    // bit1=X,           capture overflow status
    // bit0=0,           clear capture/compare interrupt pending
    // TIMER_A0->CCTL[0] = 0x0010;
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;

    /*
     * The input, CCR0Value, represents the value for CCR0 that corresponds to
     * the desired period. To compute the value for CCR0, use the equation:
     *
     *          CCR0 + 1
     * period = --------, f = SMCLK frequency/(Timer A clock divider)
     *             f
     *
     * Therefore, for SMCLK frequency = 12 MHz (see Clock.h), and the Timer A clock divider
     * equal to 24, f = 500 kHz which corresponds to T = 2 microseconds.  Therefore,
     * CCR0 = (desired period)/(2 microseconds) - 1.
     */
    TIMER_A0->CCR[0] = (CCR0Value - 1);

    TIMER_A0->EX0 = 0x0005;    // Timer A Expansion Register, clock divider /6

    // interrupts enabled in the main program after all devices initialized
    PRIORITY_REGISTER(2,0xFFFFFF00,0x00000040);
    NVIC_INTERRUPT_ENABLE(0,8);
    //NVIC->IP[2] = (NVIC->IP[2]&0xFFFFFF00)|0x00000040; // priority 2
    //NVIC->ISER[0] = 0x00000100;   // enable interrupt 8 in NVIC

    TIMER_A0->CTL |= TIMER_A_CTL_MC__UP;
    //TIMER_A0->CTL |= 0x0014;      // reset and start Timer A1 in up mode
}


// ------------TimerA0_Stop------------
// Deactivate the interrupt running a user task periodically.
// Input: none
// Output: none
void StopTimerA0(void)
{
    TIMER_A0->CTL = (TIMER_A_CTL_MC__STOP | TIMER_A_CTL_CLR);
    NVIC_INTERRUPT_DISABLE(0,8);
    //TIMER_A0->CTL &= ~0x0030;       // halt Timer A0
    //NVIC->ICER[0] = 0x00000100;     // disable interrupt 8 in NVIC
}


void TA0_0_IRQHandler(void)
{
  TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;    // acknowledge capture/compare interrupt 0
  (*TimerA0Task)();                             // execute user task
}
