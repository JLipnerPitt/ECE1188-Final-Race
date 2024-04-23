#include "Reflectance.h"
#include <msp.h>
#include "Clock.h"

void Reflectance_Init(void){

    // P5.3 configured as output
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;
    P5->DIR |= 0x08;
    P5->OUT &= ~0x08;

    // P9.2 configured as output
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04;
    P9->OUT &= ~0x04;

    // setting up 10 us duty cycle for IR sensor
    TIMER_A1->CCTL[0] = 0x0080;      // CCI0 toggle
    TIMER_A1->CCR[0] = 10000;       // Period is 2*period*8*83.33ns is 1.333*period
    TIMER_A1->EX0 = 0x0000;          // divide by 1
    TIMER_A1->CCTL[1] = 0x0040;      // CCR1 toggle/reset
    TIMER_A1->CCR[1] = 0;        // CCR1 duty cycle is duty/period
    TIMER_A1->CTL = 0x02F0;          // SMCLK=12MHz, divide by 8, up-down mode

    // sensor line configured as input
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR &= ~0xFF;
    P7->OUT &= ~0xFF;
}

void Reflectance_Start(void) {
    P5->OUT |= 0x08; // turns on even IR sensors
    P9->OUT |= 0x04; // turns on odd IR sensors

    P7->DIR |= 0xFF; // makes P7 output
    P7->OUT |= 0xFF; // starts charging capacitors

    P7->DIR &= ~0xFF; // makes P7 input
    TIMER_A1->CCR[1] = 0.001; // 10 us duty cycle
    Reflectance_End();
}

uint8_t Reflectance_End(void) {
    TIMER_A1->CCR[1] = 0;
    uint8_t input = ~(P7->IN);
    P5->OUT &= ~0x08; // turns off even IR sensors
    P9->OUT &= ~0x04; // turns on even IR sensors
    if (input != 0xFF) {
        while(1) {

        }
    }
    return input;
}


