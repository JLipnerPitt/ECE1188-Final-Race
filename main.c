#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/opt3101.h"
#include "../inc/LaunchPad.h"
#include "../inc/UART0.h"
#include "SysTickInts.h"
#include "Motor.h"
#include "PWM.h"
#include "Tachometer.h"
#include "Reflectance.h"

void UartSetCur(uint8_t newX, uint8_t newY)
{
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec


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
int32_t DistErrorR;
int32_t DistErrorL;
uint32_t DistError;
uint32_t DistReference = 0;
uint32_t DistAdjustment;
int32_t TempDR, TempDL, TempD_Error = 0;
uint32_t LeftDuty, RightDuty = 5000;
uint8_t semaphore = 0;
int32_t UR,UL;

void SysTick_Handler(void) {

    if (semaphore == 2) {
        Reflectance_Start();
    }
    else if (semaphore == 3) {
        Reflectance_End();
        DistError = Distances[0] - Distances[2];
        DistAdjustment = DistError/2;

        UL = (UL+(Kp*DistAdjustment) + (Ki*DistAdjustment/1024)+ (Kd*(DistAdjustment - TempDL)));
        UR = (UR+(Kp*DistAdjustment) + (Ki*DistAdjustment/1024)+ (Kd*(DistAdjustment - TempDR)));

        if (Distances[1] < 250) {
            if (Distances[0] < Distances[2]) {
                Motor_Right(UL,0);
            }
            else if (Distances[2] < Distances[0]) {
                Motor_Left(0,UR);
            }
            else {
                Motor_Right(UL, 0);
            }
        }
        else {

            if (Distances[0] < 150 && Distances[2] > 175) {
                Motor_Forward(UL,UR);
            }
            else if (Distances[2] < 150 && Distances[0] > 175) {
                Motor_Forward(UL,UR);
            }
            else {
                Motor_Forward(5000,5000);
            }
        }

        TempDL = DistAdjustment;
        TempDR = DistAdjustment;
        semaphore = 0;
        return;

    }
    semaphore++;
}
void main(void)
{ // busy-wait implementation

  Clock_Init48MHz();
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  Motor_Init();
  Reflectance_Init();
  Tachometer_Init();
  //Init();
  //Clear();
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_StartMeasurementChannel(channel);
  EnableInterrupts();
  StartTime = SysTick->VAL;
  SysTick_Init(48000,2);
    while(1)
    {
      UR = 5000;
      UL = 5000;
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
