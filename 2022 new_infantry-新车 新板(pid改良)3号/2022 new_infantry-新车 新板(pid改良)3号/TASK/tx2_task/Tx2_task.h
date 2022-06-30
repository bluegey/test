/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       Tx2_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note       Ԥ���룺��������ֵ��PID��
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020     		czw              1. ���
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
//ԭ�ṹ��
typedef __packed struct
{
    unsigned char head;
    float angle_pitch;
    float angle_yaw;
	  float distance;
	  int shoot_flag;//�����־  1���� 0ͣ��
    unsigned char CRC8;
} PC_Ctrl_t;



//��λ������ת��������
typedef union
{
    PC_Ctrl_t PcDate;
    unsigned char PcDataArray[sizeof(PC_Ctrl_t)];
} PC_Ctrl_Union_t;

typedef struct
{
    float yaw;
    float pitch;
    u8 mode;//ģʽ 0��������� 1���з�С���� 2��С�糵 3����糵
    u8 color;//��ɫ
    u8 shoot_speed;//����
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















