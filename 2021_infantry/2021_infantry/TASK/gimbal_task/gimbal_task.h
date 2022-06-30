/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020		     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "start_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"

#include "remote_control.h"
#include "CAN_Receive.h"
#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"
#include "Tx2_task.h"
#include "pid.h"
#include "arm_math.h"
#include "user_lib.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "Ramp_Control.h"
#include "detect_task.h"
#include "Remote_Control.h"
//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 	50
#define gimbal_debug_start 		0
//��̨��������
#define GIMBAL_CONTROL_TIME 1
/***************************************************************************/
//��������
#define INFANTRY_ID   4
#if (INFANTRY_ID==3)
    #define Glimbal_Yaw_Offset  							5471
    #define Glimbal_Yaw_Offset_Back 		      1375
    #define Glimbal_Pitch_Offset 							4080
    #define PITCH_TURN     									  -1  //����Ƿ�װ��   1��װ  2��װ
    #define PWM  															17838
		#define 	PITCH_MIN  			-35
    #define 	PITCH_MAX   		 25
#elif (INFANTRY_ID==4)
    #define Glimbal_Yaw_Offset  							2083
    #define Glimbal_Yaw_Offset_Back 		      2689
    #define Glimbal_Pitch_Offset 							2679
    #define PITCH_TURN     									  -1
    #define PWM  															17915
		#define 	PITCH_MIN  			-28
    #define 	PITCH_MAX   		 20
#elif (INFANTRY_ID==5)
    #define Glimbal_Yaw_Offset  							721
    #define Glimbal_Yaw_Offset_Back 		      4817
    #define Glimbal_Pitch_Offset 							4171
    #define PITCH_TURN    									  -1
    #define PWM  															17915
		#define 	PITCH_MIN  			-20
    #define 	PITCH_MAX   		25
#endif


//�������޷� ��λ ��

//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define 	YawChannel 			2
#define 	PitchChannel 		3
//����Ƿ�װ
//#define 	PITCH_TURN 			0
//#define 	YAW_TURN 			0
//remote control parameters
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.00030f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.00035f

#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.0030f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 			0.0045f
/***********************PID���������������******************************/
//YAW 	�޷�
#define YAW_PID_MAX_OUT 										1000.0f
#define YAW_PID_MAX_IOUT 									1000.0f
#define YAW_SPEED_PID_MAX_OUT 			29999.0f
#define YAW_SPEED_PID_MAX_IOUT 			18000.0f
//pitch �޷�
#define PITCH_PID_MAX_OUT 									1000.0f
#define PITCH_PID_MAX_IOUT 								1000.0f
#define PITCH_SPEED_PID_MAX_OUT 		29999.0f
#define PITCH_SPEED_PID_MAX_IOUT 		18000.0f
/*************************************��ʼ��PID����**************************************/
//��ʼ��pitch �ǶȻ� PID����
#define PITCH_INIT_PID_KP						15.0f
#define PITCH_INIT_PID_KI						0.0f
#define PITCH_INIT_PID_KD					0.0f
//��ʼ��pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP 			160.0f
#define PITCH_SPEED_PID_KI 				0.0f
#define PITCH_SPEED_PID_KD 			0.0f
//��ʼ��yaw �ǶȻ� PID����
#define YAW_INIT_PID_KP							16.0f
#define YAW_INIT_PID_KI							0.0f
#define YAW_INIT_PID_KD						0.0f
//��ʼ��yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP 				120.0f
#define YAW_SPEED_PID_KI 					0.0f
#define YAW_SPEED_PID_KD 				0.0f
/*************************************�ֿ�PID����**************************************/
//�ֶ�ģʽpitch �ǶȻ� PID����

//�ٶȻ�
#if (INFANTRY_ID==3)
//�ֶ�ģʽ
#define PITCH_MANUAL_ANGLE_PID_KP		20.0f
#define PITCH_MANUAL_ANGLE_PID_KI		0.0f
#define PITCH_MANUAL_ANGLE_PID_KD	0.0f
#define PITCH_MANUAL_SPEED_PID_KP 		150.0f   
#define PITCH_MANUAL_SPEED_PID_KI			0.0f
#define PITCH_MANUAL_SPEED_PID_KD		0.0f
//����ģʽ
#define YAW_TRACK_ANGLE_PID_KP			15.0f// 3�� 11
#define YAW_TRACK_ANGLE_PID_KI				0.05f//0.01
#define YAW_TRACK_ANGLE_PID_KD			0.0f
//�ٶȻ�
#define YAW_TRACK_SPEED_PID_KP				100.0f//100
#define YAW_TRACK_SPEED_PID_KI				0.0f
#define YAW_TRACK_SPEED_PID_KD			0.0f

#define PITCH_TRACK_ANGLE_PID_KP		11.0f
#define PITCH_TRACK_ANGLE_PID_KI		0.01f
#define PITCH_TRACK_ANGLE_PID_KD		0.0f
//�ٶȻ�
#define PITCH_TRACK_SPEED_PID_KP 		150.0f
#define PITCH_TRACK_SPEED_PID_KI			0.0f
#define PITCH_TRACK_SPEED_PID_KD		0.0f
#elif ((INFANTRY_ID==4)||(INFANTRY_ID==5))
//�ֶ�ģʽ
#define PITCH_MANUAL_ANGLE_PID_KP		20.0f
#define PITCH_MANUAL_ANGLE_PID_KI		0.0f
#define PITCH_MANUAL_ANGLE_PID_KD	0.0f
#define PITCH_MANUAL_SPEED_PID_KP 		150.0f
#define PITCH_MANUAL_SPEED_PID_KI			0.0f
#define PITCH_MANUAL_SPEED_PID_KD		0.0f
//����ģʽ
#define YAW_TRACK_ANGLE_PID_KP			35.0f// 
#define YAW_TRACK_ANGLE_PID_KI				0.05f//0.1
#define YAW_TRACK_ANGLE_PID_KD			0.0f
//�ٶȻ�
#define YAW_TRACK_SPEED_PID_KP				110.0f//150
#define YAW_TRACK_SPEED_PID_KI				0.0f
#define YAW_TRACK_SPEED_PID_KD			0.0f

#define PITCH_TRACK_ANGLE_PID_KP		20.0f
#define PITCH_TRACK_ANGLE_PID_KI		0.1f
#define PITCH_TRACK_ANGLE_PID_KD		0.0f
//�ٶȻ�
#define PITCH_TRACK_SPEED_PID_KP 		150.0f
#define PITCH_TRACK_SPEED_PID_KI			0.0f
#define PITCH_TRACK_SPEED_PID_KD		0.0f
#endif




//�ֶ�ģʽyaw �ǶȻ� PID����
#define YAW_MANUAL_ANGLE_PID_KP			20.0f
#define YAW_MANUAL_ANGLE_PID_KI			0.0f
#define YAW_MANUAL_ANGLE_PID_KD		0.0f
//�ٶȻ�
#define YAW_MANUAL_SPEED_PID_KP			160.0f
#define YAW_MANUAL_SPEED_PID_KI				0.0f
#define YAW_MANUAL_SPEED_PID_KD			0.0f
/***********************************����PID����****************************************/
//����ģʽpitch �ǶȻ� PID����

//����ģʽyaw �ǶȻ� PID����

/**********************************��������PID����*****************************************/
//���ģʽpitch �ǶȻ� PID����
#define PITCH_STBUF_ANGLE_PID_KP		30.0f//20
#define PITCH_STBUF_ANGLE_PID_KI			0.0f
#define PITCH_STBUF_ANGLE_PID_KD		0.0f
//����ٶȻ�
#define PITCH_STBUF_SPEED_PID_KP 		130.0f//150
#define PITCH_STBUF_SPEED_PID_KI				0.0f
#define PITCH_STBUF_SPEED_PID_KD			0.0f
//���ģʽyaw �ǶȻ� PID����
#define YAW_STBUF_ANGLE_PID_KP			30.0f//20
#define YAW_STBUF_ANGLE_PID_KI				0.0f
#define YAW_STBUF_ANGLE_PID_KD			0.0f
//����ٶȻ�
#define YAW_STBUF_SPEED_PID_KP				150.0f//160
#define YAW_STBUF_SPEED_PID_KI					0.0f
#define YAW_STBUF_SPEED_PID_KD				0.0f
/***************************************************************************/
/************************** gimbal parameter *****************************/
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO    	(8192.0f/360.0f)
/* Reduction ratio of pitch axis motor *//*pitch�����ļ��ٱ�*/
#define PIT_DECELE_RATIO       					1.0f
/* Reduction ratio of yaw axis motor *//*yaw�����ļ��ٱ�*/
#define YAW_DECELE_RATIO       				1.0f
/* the positive direction of pitch axis motor *//*pitch��������*/
#define PIT_MOTO_POSITIVE_DIR  		1.0f
/* the positive direction of yaw axis motor *//*yaw����������*/
#define YAW_MOTO_POSITIVE_DIR  1.0f
/* the positive direction of tirgger motor *//*���������*/
#define TRI_MOTO_POSITIVE_DIR  		1.0f


typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Gimbal_PID_t;

typedef enum
{
    GIMBAL_ZERO_FORCE = 0, 			//��̨���� 0
    GIMBAL_INIT,           									//��̨��ʼ��(����) 1
    GIMBAL_MANUAL_MODE,   	 	//��̨�ֶ����� 2
    GIMBAL_RELATIVE_MODE, 		 //��̨���̷���ģʽ 3
    GIMBAL_TRACK_ARMOR,			 //��̨������� 4
    GIMBAL_SHOOT_BUFF,					 //��̨������� 5
    GIMBAL_PATROL_MODE,    		//��̨������� 6
    GIMBAL_DEBUG_MODE, 				//��̨����ģʽ 7
    GIMBAL_MOTIONLESS,					//ң�������� 8
    GIMBAL_SHOOT_DOGE,					//����С���� 9
    GIMBAL_LOADING_BULLET,	//װ�� 10
    GIMBAL_TURN180,									//��̨yawŤ180�� 11
} gimbal_behaviour_e;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;//�������
    const Encoder_process_t *gimbal_encoder_measure;//����������
    Gimbal_PID_t gimbal_motor_angle_pid;
    PidTypeDef gimbal_motor_gyro_pid;
    uint16_t offset_ecd;
    fp32 relative_angle;
    fp32 absolute_angle;
    fp32 motor_gyro;
    fp32 motor_gyro_set;
    fp32 gimbal_angle_set;
	  fp32 gimbal_for_angle_set;
	  fp32 gimbal_turn_around;
    fp32 current_set;       //ͨ��pid���㣬����current������give_current
    int16_t given_current;  //���ֱֵ�Ӵ������

    extKalman_t Error_Kalman;//����һ����̨�Ƕ�������kalmanָ��
} Gimbal_Motor_t;

typedef struct
{
float yaw_angle;//��ǰ�Ƕ�
float	last_yaw_angle;//
float yaw_speed;
float last_yaw_speed;
float yaw_acceleration;
float pitch_angle;
float last_pitch_angle;
float pitch_speed;
float last_pitch_speed;
float pitch_acceleration;
float yaw_for_time;
float pitch_for_time;
float now_time;
float last_time;
float time_err;
float	pitch_forecast_angle;
float	yaw_forecast_angle;
float yaw_forcast_speed;
	
}Forecast;
typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const Angular_Handle *gimbal_angle_gyro_point;
    const PC_Ctrl_Union_t *PC_Ctrl_Measure_Point;
    const monitor_t *gimbal_monitor_point;
    const ext_game_robot_status_t *gimbal_status_measure;//���̲���ϵͳ���ʶ�ȡ
    Gimbal_Motor_t gimbal_yaw_motor;
    Gimbal_Motor_t gimbal_pitch_motor;

    gimbal_behaviour_e 	gimbal_behaviour;
    gimbal_behaviour_e	last_gimbal_behaviour;
    float pitch_angle_dynamic_ref;			//�Ƕȶ�̬����
    float yaw_angle_dynamic_ref;
    float pitch_angle_PcCtrl_ref;			//PC�Ƕȶ�̬����
    float yaw_angle_PcCtrl_ref;
		float now_yaw_speed;
	float now_pitch_speed;
	float PC_RAM_pitch_ref;
	float PC_RAM_yaw_ref;
} Gimbal_Control_t;
extern void GIMBAL_task(void *pvParameters);
extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);

#endif
