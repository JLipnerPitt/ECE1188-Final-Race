#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "I2CB1.h"
#include "CortexM.h"
#include "opt3101.h"
#include "LaunchPad.h"
#include "UART0.h"
#include <stdio.h>
#include "SysTickInts.h"
#include "Motor.h"
#include "PWM.h"
#include "Tachometer.h"
#include "Bump.h"  // Include Bump header file

uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec
uint32_t channel = 1;
uint8_t MainCount = 0;

bool pollDistanceSensor(void)
{
  if(OPT3101_CheckDistanceSensor())
  {
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}

int32_t goal_distance = 0;
int32_t Error_L;
int32_t Error_R;
int32_t Ki = 31000;  // integral gain
float Kp = 14.8; // proportional gain
float Kd = 0.0000009; // derivative gain
int32_t ErrorR;
int32_t ErrorL;
int32_t DistError;
int32_t DesiredR = 250;
int32_t DesiredL = 250;
int32_t TempDR, TempDL, TempD_Error = 0;
uint32_t LeftDuty, RightDuty = 7500;
uint8_t semaphore = 0;
int32_t UR,UL;
char chat;


void SysTick_Handler(void) {
    //distance sensor logic
    if (semaphore == 2){
        ErrorL = DesiredR - Distances[0];
        ErrorR = DesiredL - Distances[2];
        DistError = ErrorL-ErrorR;
        UL = (UL+(Kp*ErrorL) + (Ki*ErrorL/1024)+ (Kd*(ErrorL - TempDL)));
        UR = (UR+(Kp*ErrorR) + (Ki*ErrorR/1024)+ (Kd*(ErrorR - TempDR)));

        if (Distances[1] < 250) {
            if (Distances[0] < Distances[2]) {
                Motor_Right(3500,0);
            }
            else if (Distances[2] < Distances[0]) {
                Motor_Left(0,3500);
            }
        }
        else {
            if (Distances[2] > 500) {
                Motor_Right(3500,0);
            }
            else if (Distances[0] < 150 && Distances[2] > 175) {
                Motor_Forward(UL,UR);
            }
            else if (Distances[2] < 150 && Distances[0] > 175) {
                Motor_Forward(UL,UR);
            }
            else {
                Motor_Forward(6750,6750);
            }
        }

        TempDL = ErrorL;
        TempDR = ErrorR;

        semaphore = 0;
        return;
    }
    semaphore++;


    //if(chat == 'g'){
     // Motor_Forward(7500, 7500);
     // LaunchPad_Output(0x06);     //sky blue = forward/go
    //}

    //if(chat == 's'){
    // Motor_Stop(0, 0);
    // LaunchPad_Output(0x07);     //white = stop
    // }
}
void main(void)
{ // busy-wait implementation

  Clock_Init48MHz();
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  UART0_Init(); // Initialize UART communication
  Motor_Init();
  Tachometer_Init();
  Bump_Init();  // Initialize bump sensors
  LaunchPad_Init();
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_StartMeasurementChannel(channel);
  EnableInterrupts();
  StartTime = SysTick->VAL;
  SysTick_Init(48000,2);
  PWM_Init(14999);
  while(1){
      //chat = UART0_InChar();

      UR = 6750;
      UL = 6750;

      if(pollDistanceSensor())
      {
        TimeToConvert = ((StartTime-SysTick->VAL)&0x00FFFFFF)/48000; // msec
        channel = (channel+1)%3;
        OPT3101_StartMeasurementChannel(channel);
        StartTime = SysTick->VAL;
      }
      WaitForInterrupt();
    }
}

