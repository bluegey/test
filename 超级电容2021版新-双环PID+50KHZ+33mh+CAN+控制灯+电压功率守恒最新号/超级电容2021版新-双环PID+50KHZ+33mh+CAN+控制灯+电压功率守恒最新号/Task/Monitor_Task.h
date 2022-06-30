#ifndef _MONITOR_TASK_H_
#define _MONITOR_TASK_H_

#include "stm32f10x.h"                  // Device header
typedef struct SystemMonitor{
  u16 CAN_Send_FrameCounter;    //CAN发送数据帧
  u16 CAN_Recive_FrameCounter;				 //接收主板数据帧
  u16 Judgement_Recive_FrameCounter;	 //接收裁判系统数据帧
  u16 IMUFrameCounter;                 //MPU6050姿态数据帧
  u16 INA260FrameCounter;              //INA260数据帧
}SystemMonitor;

extern SystemMonitor Real_Monitor;
extern SystemMonitor Rate_Monitor;
extern  uint16_t SysErrorStatus;
void Monitor_Thread1 (void const *argument);



#endif
