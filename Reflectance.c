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
    //Clock_Delay1us(1000);
    //IRSensorInput = P7->IN;
}

uint8_t Reflectance_End(void) {
    uint8_t input = ~(P7->IN);
    P5->OUT &= ~0x08; // turns off even IR sensors
    P9->OUT &= ~0x04; // turns on even IR sensors
    return input;
}

// checks how many sensors are activated
/*uint8_t Num_Of_On_Sensors(uint8_t input) {
    uint8_t i;
    uint8_t bit_number;
    uint8_t count = 0;
    bool flag = false;

    for (i=0;i<8;i++) {
        if ((input >> i) & 0x01 == 0) {
            bit_number = i;
            count++;
        }
    }
    if (count == 0) {
        return 0;
    }
    flag = true;
    return bit_number;
}*/

uint8_t decision(uint8_t input) {
    // 0x11001111 and 0x11110011 might work better in slight_left and slight_right respectively
    switch (input) {

    case 0xC3:
    case 0xE7:
    case 0xE3:
    case 0xC7:
    case 0xCE: // new
    case 0x99: // new
    return 1; // forward

    case 0xCF:
    case 0xF3:
    case 0xEF:
    case 0xF7:
    case 0x81:
    case 0x00: // new
    case 0x3C: // new
    case 0xBD: // new
    case 0x7E: // new
    case 0xDB: // new
    return 2; // slow forward

    case 0x3F:
    case 0xDC:
    case 0xDE:
    //case 0xCE:
    case 0x9C:
    case 0x1F:
    case 0x0F:
    case 0x63:
    case 0x1C:
    case 0x03:
    case 0x8F:
    case 0x87:
    case 0x9F:
    case 0xAF:
    //case 0x78: //new
    return 3; // left

    case 0xFC:
    case 0xF9:
    case 0x3B:
    case 0x7B:
    case 0x73:
    case 0x39:
    case 0xF8:
    case 0xF0:
    case 0xC6:
    case 0x38:
    case 0xC0:
    case 0xFB:
    case 0x6F:
    case 0xFA:

    //case 0xC8: // hard right?
    //case 0xE4: // right?
    //case 0xCC: // hard right?
    return 4; // right

    case 0xDF:
    case 0xF6:
    case 0x07:
    case 0x7F:
    case 0x67:
    case 0xEE:
    case 0x7C:
    return 5; // slight left

    case 0xF1:
    case 0xE1:
    case 0xE0:
    case 0xFE:
    case 0xE6: // small right?
    case 0x77:
    case 0x3E:
    return 6; // slight right

    default: return 0;
    }
}
