#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H
#include "stm32f4xx.h"
#include "string.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"
#include "CAN_Receive.h"
#include "gimbal_task.h"
#include "IMUTask.h"
#include "BSP_MPU9250_Init.h"
#include "Remote_Control.h"
#include "pid.h"
#include "arm_math.h"
#include "user_lib.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "Ramp_Control.h"
#include "get_judge_measure.h"
#include "detect_task.h"
#include "chassis_task.h"




typedef enum
{
    CHASSIS_RELAX          	= 0,//��������ģʽ
    CHASSIS_INIT           	= 1,
    CHASSIS_FOLLOW_GIMBAL  	= 2,//�Զ�������̨
    CHASSIS_SEPARATE_GIMBAL   = 3,//��̨����
    CHASSIS_DODGE_MODE        = 4,//С����
} chassis_mode_e; //��������ģʽ


typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    const Encoder_process_t *chassis_encoder_measure;
} Chassis_Motor_t;


typedef struct
{
    first_order_filter_type_t chassis_filter_set_vz;

    const Gimbal_Motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
    const Gimbal_Motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
    const ext_power_heat_data_t *chassis_power_measure;//���̲���ϵͳ���ʶ�ȡ
    const ext_game_robot_status_t *chassis_status_measure;//���̲���ϵͳ���ʶ�ȡ
    const ext_robot_hurt_t *chassis_hurt_type;
    const monitor_t *chassis_monitor_point;
    chassis_mode_e chassis_mode;               //���̿���״̬��
    Chassis_Motor_t motor_chassis[4];          //���̵������

    PidTypeDef chassis_angle_pid;              //���̸���Ƕ�pid
    PidTypeDef motor_speed_pid[4];             //���̵���ٶ�pid
    PidTypeDef dodge_pid;					   //����ģʽ���ٶ�pid

    const Angular_Handle *Gimbal_angle_gyro_point;
    const RC_ctrl_t *chassis_rc_ctrl;
    float RC_X_ChassisSpeedRef;			//���Ҷ�̬����
    float RC_Y_ChassisSpeedRef;			//ǰ��̬����
    float RC_Z_ChassisSpeedRef;			//��ת��̬����

    first_order_filter_type_t chassis_cmd_slow_set_vz;

    float vx;
    float vy;
    float vz;
    int16_t        rotate_x_offset;
    int16_t        rotate_y_offset;
    float            wheel_spd_ref[4];
    float						wheel_spd_fdb[4];
    float           	chassis_relative_angle_set;
    float						Chassis_Gyro_Error;//���YAW�����ķ����е�Ƕ�

} chassis_move_t;

extern void mecanum_calc(float vx, float vy, float vz, float speed[], chassis_move_t *power_ctrl); //ȫ���㷨

//���ʿ��Ʒ���
void Chassis_Power_Limit(chassis_move_t *power_ctrl);
//�������ݿ��Ʒ���
void Super_power_ctrl(chassis_move_t *power_ctrl);

#endif

