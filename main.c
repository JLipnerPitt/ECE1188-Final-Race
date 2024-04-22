#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "I2CB1.h"
#include "CortexM.h"
#include "opt3101.h"
#include "LaunchPad.h"
#include "SysTickInts.h"
#include "PWM.h"
#include "Tachometer.h"
#include "UART0.h" // Include UART0 header file
#include "Motor.h" // Include Motor header file
#include "Bump.h"  // Include Bump header file

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
int32_t ErrorR;
int32_t ErrorL;
int32_t DistError;
int32_t DesiredR = 250;
int32_t DesiredL = 250;
int32_t TempDR, TempDL, TempD_Error = 0;
uint32_t LeftDuty, RightDuty = 5000;
uint8_t semaphore = 0;
int32_t UR,UL;
char chat;

void SysTick_Handler(void) {


    if(chat == 'g'){
        Motor_Forward(7500, 7500);
        LaunchPad_Output(0x06);     //sky blue = forward/go
    }

    if(chat == 'b'){
        Motor_Backward(7500, 7500);
        LaunchPad_Output(0x05);     //pink = backward
    }

    if(chat == 's'){
        Motor_Stop(0, 0);
        LaunchPad_Output(0x07);     //white = stop
    }

    if (semaphore == 3) {
        ErrorL = DesiredR - Distances[0];
        ErrorR = DesiredL - Distances[2];
        DistError = ErrorL-ErrorR;
        UL = (UL+(Kp*ErrorL) + (Ki*ErrorL/1024)+ (Kd*(ErrorL - TempDL)));
        UR = (UR+(Kp*ErrorR) + (Ki*ErrorR/1024)+ (Kd*(ErrorR - TempDR)));

        if (Distances[1] < 100) {
            if (Distances[0] < Distances[2]) {
                Motor_Right(UL,0);
            }
            else if (Distances[2] < Distances[0]) {
                Motor_Left(0,UR);
            }
        }
        else {

            if (Distances[0] < 125 && Distances[2] > 175) {
                Motor_Forward(UL,UR);
            }
            else if (Distances[2] < 125 && Distances[0] > 175) {
                Motor_Forward(UL,UR);
            }
            else {
                Motor_Forward(3500,3500);
            }
        }

        TempDL = ErrorL;
        TempDR = ErrorR;

        semaphore = 0;
        Clock_Delay1ms(10);
    }
    semaphore++;
}
void main(void)
{ // busy-wait implementation

  Clock_Init48MHz();
  I2CB1_Init(30); // baud rate = 12MHz/60=200kHz
  UART0_Init(); // Initialize UART communication
  Motor_Init(); // Initialize motor ports
  Bump_Init();  // Initialize bump sensors
  LaunchPad_Init();
  Tachometer_Init();

  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_StartMeasurementChannel(channel);

  EnableInterrupts();
  StartTime = SysTick->VAL;
  SysTick_Init(48000,2);
  PWM_Init(14999);

  while(1)
    {
      chat = UART0_InChar(); // Receive command from Bluetooth

      UR = DesiredR;
      UL = DesiredL;

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
