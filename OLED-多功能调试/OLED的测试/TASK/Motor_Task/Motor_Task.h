#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H
#include "sys.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"

#include "OLED_Task.h"
#include "Can_Receive.h"
#include "pid.h"
#define Motor_Speed_PID_KP 10.00f
#define Motor_Speed_PID_KI 0.00f
#define Motor_Speed_PID_KD 0.00f
#define Motor_Speed_PID_MAX_Out 10000.0
#define Motor_SPeed_PID_MAX_IOUT 1000.0

#define Motor_AngleOR_PID_KP 10.00f//外环KP
#define Motor_AngleOR_PID_KI 0.00f
#define Motor_AngleOR_PID_KD 0.00f
#define Motor_AngleOR_PID_MAX_Out 10000.0
#define Motor_AngleOR_PID_MAX_IOUT 1000.0

#define Motor_AngleIR_PID_KP 2.00f//外环KP
#define Motor_AngleIR_PID_KI 0.00f
#define Motor_AngleIR_PID_KD 0.00f
#define Motor_AngleIR_PID_MAX_Out 10000.0
#define Motor_AngleIR_PID_MAX_IOUT 1000.0

typedef struct
{
	const motor_measure_t *get_Motor_3508_Measure;
//	const Encoder_process_t *get_Motor_3508_Encoder;
}Motor_t;
typedef  struct
{
	PidTypeDef PID_SPEED;
	fp32 Motor_Speed_Set;
	fp32 Motor_Speed_Fdb;
	int16_t Give_Speed_Current;
}Motor_Speed;//电机速度模式
typedef struct
{
	PidTypeDef PID_Outer;//外环
	PidTypeDef PID_Inner;//内环
	fp32 Motor_Angle_Set;
	fp32 Motor_Angle_Fdb;
	fp32 Motor_Speed_Fdb;
	int16_t Give_Angle_Current;
}Motor_Angle;//电机角度模式
typedef struct
{
	Motor_t M3608_Date[4];
	Motor_Speed M3508_Speed[4];
	Motor_Angle M3508_Angle[4];
}Motor_3508;
extern void Motor_Task(void *pvParameters);


#endif
