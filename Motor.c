// Motor.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot.

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include <stdint.h>
#include "msp.h"
#include "CortexM.h"
#include "PWM.h"
#include "Motor.h"

// ------------Motor_Init------------
// Initialize GPIO pins for output, which will be
// used to control the direction of the motors and
// to enable or disable the drivers.
// The motors are initially stopped, the drivers
// are initially powered down, and the PWM speed
// control is uninitialized.
// Input: none
// Output: none
void Motor_Init(void){
    // Initialize P2.6, P2.7 as outputs for motor PWM
    PWM_Init(14999); // initialize right motor and right motor PWM to 0%

    // Initialize P5.4 and P5.5 as outputs for motor direction
    P5->SEL0 &= ~0x30; // Clear SEL0 bits
    P5->SEL1 &= ~0x30; // Clear SEL1 bits
    P5->DIR |= 0x30;   // Set P5.4 and P5.5 as outputs
    P5->DS |= 0xC0;    // Activate increase drive strength
    P5->OUT &= ~0x30;  // Set direction pins low initially

    // Initialize P3.6 and P3.7 as outputs for motor enable
    P3->SEL0 &= ~0xC0; // Clear SEL0 bits
    P3->SEL1 &= ~0xC0; // Clear SEL1 bits
    P3->DIR |= 0xC0;   // Set P3.6 and P3.7 as outputs
    P3->DS |= 0xC0;    // Activate increase drive strength
    P3->OUT &= ~0xC0;  // Set enable pins low initially
}

// ------------Motor_Stop------------
// Stop the motors, power down the drivers, and
// set the PWM speed control to 0% duty cycle.
// Input: none
// Output: none
void Motor_Stop(void){

    P5->OUT &= ~0x30; // Clear direction bits
    P3->OUT &= ~0xC0;  // Disable drivers by clearing enable bits

    PWM_Duty(0, 0); // Set right motor and right motor PWM to 0%
}

// ------------Motor_Forward------------
// Drive the robot forward by running left and
// right wheels forward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty){

    P5->OUT &= ~0x30;   // Clear direction bits for forward motion
    P3->OUT |= 0xC0;    // Enable both drivers

    PWM_Duty(leftDuty,rightDuty); // Set PWM duty cycles
}

// ------------Motor_Right------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty){ 

    // Left motor forward
    P5->OUT &= ~0x10;   // set P5.4 low (direction forward)
    P3->OUT |= 0x80;    // set P3.7 high (enable motor)

    // Right motor backward
    P5->OUT |= 0x20;    // set P5.5 high (direction backward)
    P3->OUT |= 0x40;    // set P3.6 high (enable motor)

    PWM_Duty(leftDuty,rightDuty); // Set PWM duty cycles
}

// ------------Motor_Left------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty){ 

    // Left motor backward
    P5->OUT |= 0x10;    // set P5.4 high (direction backward)
    P3->OUT |= 0x80;    // set P3.7 high (enable motor)

    // Right motor forward
    P5->OUT &= ~0x20;   // set P5.5 low (direction forward)
    P3->OUT |= 0x40;    // set P3.6 high (enable motor)

    PWM_Duty(leftDuty,rightDuty); // Set PWM duty cycles
}

// ------------Motor_Backward------------
// Drive the robot backward by running left and
// right wheels backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty){

    // Left motor backward
    P5->OUT |= 0x10;    // set P5.4 high (direction backward)
    P3->OUT |= 0x80;    // set P3.7 high (enable motor)

    // Right motor backward
    P5->OUT |= 0x20;    // set P5.5 high (direction backward)
    P3->OUT |= 0x40;    // set P3.6 high (enable motor)

    PWM_Duty(leftDuty,rightDuty); // Set PWM duty cycles
}
