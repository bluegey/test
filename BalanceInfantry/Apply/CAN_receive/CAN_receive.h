/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  * @note       ���ļ�����freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef CANTASK_H
#define CANTASK_H
#include "stm32f4xx.h"

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x206,
    CAN_PIT_MOTOR_ID = 0x205,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    CAN_TRIGGER_MOTOR_ID    = 0x201,
    CAN_FRICTION_right_ID   = 0x202,
    CAN_FRICTION_left_ID    = 0x203,

} can_msg_id_e;
//rm���ͳһ���ݽṹ��
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	int32_t  count;
    int32_t all_ecd;
} motor_measure_t;
//�������ݽṹ��
typedef struct
{
    uint16_t 	volt;
    uint16_t	power;
    uint16_t	current;
} Super_power_t;
typedef struct 
{
  uint16_t power;
	uint16_t flag;
	uint16_t buffer_power;
}Send_Data;
typedef union 
{
  Send_Data TX_data;
  uint8_t Array_Tx_data[sizeof(Send_Data)];
}Tx_Union_data;
//������̨�����������revΪ�����ֽ�
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//���͵��̵����������
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//�������ģ���������
extern void CAN_CMD_Shoot(int16_t trigger, int16_t fri_right, int16_t fri_left);
//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//����trigger,friction���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
extern const motor_measure_t *get_Friction_Motor_Measure_Point(uint8_t i);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

#define  RATE_BUF_SIZE 5
typedef struct
{
    int32_t diff;
    int32_t round_cnt;
    int32_t ecd_raw_rate;
    int32_t rate_buf[RATE_BUF_SIZE]; 	//buf��for filter
    uint8_t buf_count;					//�˲�����buf��
    int32_t filter_rate;				//�ٶ�
} Encoder_process_t;
//���ص��̵��������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const Encoder_process_t *get_Chassis_Encoder_Measure_Point(uint8_t i);
//����Ħ�����������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const Encoder_process_t *get_Friction_Encoder_Measure_Point(uint8_t i);
//����trigger������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const Encoder_process_t *get_Trigger_Encoder_Measure_Point(void);
//����yaw������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const Encoder_process_t *get_Yaw_Gimbal_Encoder_Measure_Point(void);
//����pit������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const Encoder_process_t *get_Pitch_Gimbal_Encoder_Measure_Point(void);

//������������ģ��
extern Super_power_t Super_power;
/*
  �������������б������ܺ͵�ֵ�ļ���
*/
extern void EncoderProcess3508(Encoder_process_t* v, motor_measure_t *motor);
extern void EncoderProcess6020(Encoder_process_t* v, motor_measure_t *motor);
void CAN_CMD_SUPERPOWER(int16_t power, int16_t i,uint16_t buffer_power);
#endif



