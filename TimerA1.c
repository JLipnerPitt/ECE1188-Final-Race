// TimerA1.c
// Runs on MSP432
// Use Timer A1 in periodic mode to request interrupts at a particular
// period.

#include <stdint.h>
#include "msp.h"

void (*TimerA1Task)(void);   // user function

// ***************** TimerA1_Init ****************
// Activate Timer A1 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (24/SMCLK), 16 bits
// Outputs: none
// With SMCLK 12 MHz, period has units 2us
void TimerA1_Init(void(*task)(void), uint16_t period){
  TimerA1Task = task;             // user function
  TIMER_A1->CTL &= ~0x0030;       // halt Timer A1
  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=10,       input clock divider /4
  // bits5-4=00,       stop mode
  // bit3=X,           reserved
  // bit2=0,           set this bit to clear
  // bit1=0,           no interrupt on timer
  TIMER_A1->CTL = 0x0280;
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
  TIMER_A1->CCTL[0] = 0x0010;
  TIMER_A1->CCR[0] = (period - 1);   // compare match value
  TIMER_A1->EX0 = 0x0005;    // configure for input clock divider /6
// interrupts enabled in the main program after all devices initialized
  NVIC->IP[2] = (NVIC->IP[2]&0xFF00FFFF)|0x00400000; // priority 2
  NVIC->ISER[0] = 0x00000400;   // enable interrupt 10 in NVIC
  TIMER_A1->CTL |= 0x0014;      // reset and start Timer A1 in up mode
}


// ------------TimerA1_Stop------------
// Deactivate the interrupt running a user task periodically.
// Input: none
// Output: none
void TimerA1_Stop(void){
  TIMER_A1->CTL &= ~0x0030;       // halt Timer A1
  NVIC->ICER[0] = 0x00000400;     // disable interrupt 10 in NVIC
}


void TA1_0_IRQHandler(void){
  TIMER_A1->CCTL[0] &= ~0x0001; // acknowledge capture/compare interrupt 0
  (*TimerA1Task)();             // execute user task
}
