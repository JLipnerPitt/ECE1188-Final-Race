// BumpInt.c
// Runs on MSP432, interrupt version
// Provide low-level functions that interface bump switches on the robot.


// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include "msp.h"
#include "UART0.h"
#include "Clock.h"
#include "Motor.h"

// Global variable and semaphore for bump sensor result
volatile uint8_t bumpSensorResult;
volatile uint8_t bumpSensorSemaphore;
volatile uint8_t status;

// Function pointer for user-supplied collision handling function
/*typedef void (*CollisionHandler)(uint8_t);
static CollisionHandler collisionHandler = 0;*/

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
//void BumpInt_Init(void(*task)(uint8_t)){

void Bump_Init(void){
    // Configure bump sensor pins as inputs with internal pull-up resistors
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;
    P4->DIR &= ~0xED; // configured as input
    P4->REN |= 0xED; // enable pull resistors
    P4->OUT |= 0xED; // enable pull-up resistors

    // Save the user-supplied collision handling function
    //collisionHandler = task;

    //commented out for lab5
    // Configure and enable interrupts for bump sensors
    P4->IES = 0xED;    // Interrupt on falling edge for P4.7, P4.6, P4.5, P4.3, P4.2, P4.0
    P4->IFG = 0x00;    // Clear interrupt flags
    P4->IE = 0xED;     // Enable interrupts for P4.7, P4.6, P4.5, P4.3, P4.2, P4.0

    //NVIC_EnableIRQ(PORT4_IRQn);  // Enable the interrupt in NVIC

    //commented out for lab5
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00800000; // priority 2
    NVIC->ISER[1] = 0x00000040;        // enable interrupt 35 in NVIC

    //bumpSensorResult = 0;


    //creating LED indicator for trigger
    P1->SEL0 &= ~0x01;
    P1->SEL1 &= ~0x01;                 // configure built-in LED1 as GPIO
    P1->DIR |= 0x01;                   // make built-in LED1 out
    P1->OUT &= ~0x01;
}

// Checks if any bump sensor is activated
uint8_t Bump_Read(void){
    return P4->IN & 0xED;
}

// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    // Read the 6-bit value from the sensors
    status = P4->IV;
    if(status != 0x00) {
        while(Bump_Read() != 0xED){
            //  3,750/15,000  = 25% Duty Cycle
            Motor_Stop(0, 0);
        }
    }
}
