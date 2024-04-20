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
int32_t DistR;
int32_t DistL;
int32_t DistError;
int32_t DesiredDistR = 250;
int32_t DesiredDistL = 250;
int32_t TempDR, TempDL, TempD_Error = 0;
uint32_t LeftDuty, RightDuty = 7500;
uint32_t AdjustedL, AdjustedR = 0;

void main(void)
{ // busy-wait implementation

  Clock_Init48MHz();
  I2CB1_Init(60); // baud rate = 12MHz/60=200kHz
  Motor_Init();
  Tachometer_Init();
  //Init();
  //Clear();
  OutString("OPT3101");
  SetCursor(0, 1);
  OutString("Left =");
  SetCursor(0, 2);
  OutString("Centr=");
  SetCursor(0, 3);
  OutString("Right=");
  SetCursor(0, 4);
  OutString("Busy-wait");
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_StartMeasurementChannel(channel);
  StartTime = SysTick->VAL;
  uint8_t semaphore = 0;
  int32_t UR,UL;
    while(1)
    {
      if(pollDistanceSensor())
      {
        TimeToConvert = ((StartTime-SysTick->VAL)&0x00FFFFFF)/48000; // msec
        channel = (channel+1)%3;
        UR = RightDuty;
        UL = LeftDuty;
        DistL = DesiredDistR - Distances[0];
        DistR = DesiredDistL - Distances[2];
        DistError = DistL-DistR;
        UL = ((Kp*DistL) + (Ki*DistL/1024)+ (Kd*(DistL - TempDL)));
        UR = ((Kp*DistR) + (Ki*DistR/1024)+ (Kd*(DistR - TempDR)));

        if (Distances[0] < 50 && Distances[2] > 200) {
            Motor_Forward(UL+LeftDuty+1000,RightDuty-UR);
        }
        else if (Distances[2] < 50 && Distances[0] > 200) {
            Motor_Forward(LeftDuty-UL+1000,RightDuty+UR);
        }
        else {
            Motor_Forward(3500,3500);
        }

        TempDL = DistL;
        TempDR = DistR;

        semaphore++;
        Clock_Delay1ms(10);
        OPT3101_StartMeasurementChannel(channel);
        StartTime = SysTick->VAL;
      }
    }
  }
