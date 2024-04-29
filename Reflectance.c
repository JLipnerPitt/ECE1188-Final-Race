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

    Clock_Delay1us(10); // wait 10 us
    P7->DIR &= ~0xFF; // makes P7 input
    Clock_Delay1us(1000);
}

uint8_t Reflectance_End(void) {
    uint8_t input = ~(P7->IN);
    P5->OUT &= ~0x08; // turns off even IR sensors
    P9->OUT &= ~0x04; // turns on even IR sensors
    return input;
}

// checks how many sensors are activated
uint8_t Num_Of_On_Sensors(uint8_t input) {
    uint8_t i;
    uint8_t count = 0;

    for (i=0;i<8;i++) {
        if (((input >> i) & 0x01) == 1) {  // Added parentheses around (input >> i) & 0x01
                    count++;
        }
    }
    if (count == 0) {
        return 0;
    }
    return count;
}
