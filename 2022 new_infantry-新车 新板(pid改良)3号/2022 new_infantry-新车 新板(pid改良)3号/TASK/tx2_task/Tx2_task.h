/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       Tx2_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       预编译：编码器中值，PID。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020     		czw              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */
#ifndef _TX2_TASK_H_
#define	_TX2_TASK_H_
#include "stm32f4xx.h"
#include <stdio.h>
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"

#include "get_judge_measure.h"
#include "CAN_Receive.h"
#include "Remote_Control.h"
//原结构体
typedef __packed struct
{
    unsigned char head;
    float angle_pitch;
    float angle_yaw;
	  float distance;
	  int shoot_flag;//射击标志  1发射 0停火
    unsigned char CRC8;
} PC_Ctrl_t;



//上位机数据转换共用体
typedef union
{
    PC_Ctrl_t PcDate;
    unsigned char PcDataArray[sizeof(PC_Ctrl_t)];
} PC_Ctrl_Union_t;

typedef struct
{
    float yaw;
    float pitch;
    u8 mode;//模式 0：辅助射击 1：敌方小陀螺 2：小风车 3：大风车
    u8 color;//颜色
    u8 shoot_speed;//射速
    unsigned char CRC8;
} Send_Tx2_t;

typedef union
{
    float Yaw_absolute_angle;
    float Pitch_relative;
    unsigned char angle_data[8];
}
PC_Send_data;

typedef struct
{

	const ext_game_robot_status_t		*GameRobot_State_measure;
	const ext_shoot_data_t				*gameshoot_state_measure;
	const RC_ctrl_t *RC_pc;
	float now_pc_yaw_ref;
	float now_pc_pitch_ref;
	float last_pc_yaw_ref;
	float last_pc_pitch_ref;
} Inter_Data_t;

extern Send_Tx2_t TX_vision_Mes;
extern void Interactive_task(void  *pvParameters);
extern void Tx2_task(void  *pvParameters);
extern uint8_t PcDataCheck( uint8_t *pData );
extern void PcDataClean(unsigned  char * pData, int num);
extern const PC_Ctrl_Union_t *get_PC_Ctrl_Measure_Point(void);

#endif















