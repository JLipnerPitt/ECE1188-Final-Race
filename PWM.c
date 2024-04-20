// PWM.c
// Runs on MSP432
// PWM on P2.4 using TimerA0 TA0.CCR1
// PWM on P2.5 using TimerA0 TA0.CCR2
// PWM on P2.6 using TimerA0 TA0.CCR3
// PWM on P2.7 using TimerA0 TA0.CCR4
// MCLK = SMCLK = 3MHz DCO; ACLK = 32.768kHz
// TACCR0 generates a square wave of freq ACLK/1024 =32Hz
// Derived from msp432p401_portmap_01.c in MSPware


#include "msp.h"
#include "PWM.h"

//***************************PWM_Init34*******************************
// PWM outputs on P2.6, P2.7
// Inputs:  period (1.333us)
//          duty3
//          duty4
// Outputs: none
// SMCLK = 48MHz/4 = 12 MHz, 83.33ns
// Counter counts up to TA0CCR0 and back down
// Let Timerclock period T = 8/12MHz = 666.7ns
// period of P7.3 squarewave is 4*period*666.7ns
// P2.6=1 when timer equals TA0CCR3 on way down, P2.6=0 when timer equals TA0CCR3 on way up
// P2.7=1 when timer equals TA0CCR4 on way down, P2.7=0 when timer equals TA0CCR4 on way up
// Period of P2.6 is period*1.333us, duty cycle is duty3/period
// Period of P2.7 is period*1.333us, duty cycle is duty4/period
void PWM_Init(uint16_t period){

  P2->DIR |= 0xC0;          // P2.6, P2.7 output
  P2->SEL0 |= 0xC0;         // P2.6, P2.7 Timer0A functions
  P2->SEL1 &= ~0xC0;        // P2.6, P2.7 Timer0A functions
  TIMER_A0->CCTL[0] = 0x0080;      // CCI0 toggle
  TIMER_A0->CCR[0] = period-1;       // Period is 2*period*8*83.33ns is 1.333*period
  TIMER_A0->EX0 = 0x0000;          // divide by 1
  TIMER_A0->CCTL[3] = 0x0040;      // CCR3 toggle/reset
  TIMER_A0->CCR[3] = 0;        // CCR3 duty cycle is duty3/period
  TIMER_A0->CCTL[4] = 0x0040;      // CCR4 toggle/reset
  TIMER_A0->CCR[4] = 0;        // CCR4 duty cycle is duty4/period
  TIMER_A0->CTL = 0x02F0;          // SMCLK=12MHz, divide by 8, up-down mode
}

//***************************PWM_Duty3*******************************
// change duty cycle of PWM output on P2.6
// Inputs:  duty3
// Outputs: none
// period of P2.6 is 2*period*666.7ns, duty cycle is duty3/period
//***************************PWM_Duty4*******************************
// change duty cycle of PWM output on P2.7
// Inputs:  duty4
// Outputs: none// period of P2.7 is 2*period*666.7ns, duty cycle is duty2/period
void PWM_Duty(uint16_t Duty3, uint16_t Duty4){

  TIMER_A0->CCR[3] = Duty4;        // CCR4 duty cycle is duty4/period
  TIMER_A0->CCR[4] = Duty3;       // CCR3 duty cycle is duty3/period
}
