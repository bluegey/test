/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note       该文件不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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
//rm电机统一数据结构体
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
//超级电容结构体
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
//发送云台控制命令，其中rev为保留字节
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//发送底盘电机控制命令
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//发送射击模块控制命令
extern void CAN_CMD_Shoot(int16_t trigger, int16_t fri_right, int16_t fri_left);
//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回trigger,friction电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
extern const motor_measure_t *get_Friction_Motor_Measure_Point(uint8_t i);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

#define  RATE_BUF_SIZE 5
typedef struct
{
    int32_t diff;
    int32_t round_cnt;
    int32_t ecd_raw_rate;
    int32_t rate_buf[RATE_BUF_SIZE]; 	//buf，for filter
    uint8_t buf_count;					//滤波更新buf用
    int32_t filter_rate;				//速度
} Encoder_process_t;
//返回底盘电机编码器变量地址，通过指针方式获取原始数据
extern const Encoder_process_t *get_Chassis_Encoder_Measure_Point(uint8_t i);
//返回摩擦电机编码器变量地址，通过指针方式获取原始数据
extern const Encoder_process_t *get_Friction_Encoder_Measure_Point(uint8_t i);
//返回trigger编码器变量地址，通过指针方式获取原始数据
extern const Encoder_process_t *get_Trigger_Encoder_Measure_Point(void);
//返回yaw编码器变量地址，通过指针方式获取原始数据
extern const Encoder_process_t *get_Yaw_Gimbal_Encoder_Measure_Point(void);
//返回pit编码器变量地址，通过指针方式获取原始数据
extern const Encoder_process_t *get_Pitch_Gimbal_Encoder_Measure_Point(void);

//声明超级电容模块
extern Super_power_t Super_power;
/*
  采用向量法进行编码器总和的值的计算
*/
extern void EncoderProcess3508(Encoder_process_t* v, motor_measure_t *motor);
extern void EncoderProcess6020(Encoder_process_t* v, motor_measure_t *motor);
void CAN_CMD_SUPERPOWER(int16_t power, int16_t i,uint16_t buffer_power);
#endif



