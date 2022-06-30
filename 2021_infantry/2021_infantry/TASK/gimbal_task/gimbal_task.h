/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020		     RM              1. 完成
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
//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 	50
#define gimbal_debug_start 		0
//云台控制周期
#define GIMBAL_CONTROL_TIME 1
/***************************************************************************/
//归中设置
#define INFANTRY_ID   4
#if (INFANTRY_ID==3)
    #define Glimbal_Yaw_Offset  							5471
    #define Glimbal_Yaw_Offset_Back 		      1375
    #define Glimbal_Pitch_Offset 							4080
    #define PITCH_TURN     									  -1  //电机是否装反   1正装  2反装
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


//俯仰角限幅 单位 度

//yaw,pitch控制通道以及状态开关通道
#define 	YawChannel 			2
#define 	PitchChannel 		3
//电机是否反装
//#define 	PITCH_TURN 			0
//#define 	YAW_TURN 			0
//remote control parameters
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.00030f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.00035f

#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.0030f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 			0.0045f
/***********************PID最大输出，积分输出******************************/
//YAW 	限幅
#define YAW_PID_MAX_OUT 										1000.0f
#define YAW_PID_MAX_IOUT 									1000.0f
#define YAW_SPEED_PID_MAX_OUT 			29999.0f
#define YAW_SPEED_PID_MAX_IOUT 			18000.0f
//pitch 限幅
#define PITCH_PID_MAX_OUT 									1000.0f
#define PITCH_PID_MAX_IOUT 								1000.0f
#define PITCH_SPEED_PID_MAX_OUT 		29999.0f
#define PITCH_SPEED_PID_MAX_IOUT 		18000.0f
/*************************************初始化PID参数**************************************/
//初始化pitch 角度环 PID参数
#define PITCH_INIT_PID_KP						15.0f
#define PITCH_INIT_PID_KI						0.0f
#define PITCH_INIT_PID_KD					0.0f
//初始化pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP 			160.0f
#define PITCH_SPEED_PID_KI 				0.0f
#define PITCH_SPEED_PID_KD 			0.0f
//初始化yaw 角度环 PID参数
#define YAW_INIT_PID_KP							16.0f
#define YAW_INIT_PID_KI							0.0f
#define YAW_INIT_PID_KD						0.0f
//初始化yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP 				120.0f
#define YAW_SPEED_PID_KI 					0.0f
#define YAW_SPEED_PID_KD 				0.0f
/*************************************手控PID参数**************************************/
//手动模式pitch 角度环 PID参数

//速度环
#if (INFANTRY_ID==3)
//手动模式
#define PITCH_MANUAL_ANGLE_PID_KP		20.0f
#define PITCH_MANUAL_ANGLE_PID_KI		0.0f
#define PITCH_MANUAL_ANGLE_PID_KD	0.0f
#define PITCH_MANUAL_SPEED_PID_KP 		150.0f   
#define PITCH_MANUAL_SPEED_PID_KI			0.0f
#define PITCH_MANUAL_SPEED_PID_KD		0.0f
//自瞄模式
#define YAW_TRACK_ANGLE_PID_KP			15.0f// 3号 11
#define YAW_TRACK_ANGLE_PID_KI				0.05f//0.01
#define YAW_TRACK_ANGLE_PID_KD			0.0f
//速度环
#define YAW_TRACK_SPEED_PID_KP				100.0f//100
#define YAW_TRACK_SPEED_PID_KI				0.0f
#define YAW_TRACK_SPEED_PID_KD			0.0f

#define PITCH_TRACK_ANGLE_PID_KP		11.0f
#define PITCH_TRACK_ANGLE_PID_KI		0.01f
#define PITCH_TRACK_ANGLE_PID_KD		0.0f
//速度环
#define PITCH_TRACK_SPEED_PID_KP 		150.0f
#define PITCH_TRACK_SPEED_PID_KI			0.0f
#define PITCH_TRACK_SPEED_PID_KD		0.0f
#elif ((INFANTRY_ID==4)||(INFANTRY_ID==5))
//手动模式
#define PITCH_MANUAL_ANGLE_PID_KP		20.0f
#define PITCH_MANUAL_ANGLE_PID_KI		0.0f
#define PITCH_MANUAL_ANGLE_PID_KD	0.0f
#define PITCH_MANUAL_SPEED_PID_KP 		150.0f
#define PITCH_MANUAL_SPEED_PID_KI			0.0f
#define PITCH_MANUAL_SPEED_PID_KD		0.0f
//自瞄模式
#define YAW_TRACK_ANGLE_PID_KP			35.0f// 
#define YAW_TRACK_ANGLE_PID_KI				0.05f//0.1
#define YAW_TRACK_ANGLE_PID_KD			0.0f
//速度环
#define YAW_TRACK_SPEED_PID_KP				110.0f//150
#define YAW_TRACK_SPEED_PID_KI				0.0f
#define YAW_TRACK_SPEED_PID_KD			0.0f

#define PITCH_TRACK_ANGLE_PID_KP		20.0f
#define PITCH_TRACK_ANGLE_PID_KI		0.1f
#define PITCH_TRACK_ANGLE_PID_KD		0.0f
//速度环
#define PITCH_TRACK_SPEED_PID_KP 		150.0f
#define PITCH_TRACK_SPEED_PID_KI			0.0f
#define PITCH_TRACK_SPEED_PID_KD		0.0f
#endif




//手动模式yaw 角度环 PID参数
#define YAW_MANUAL_ANGLE_PID_KP			20.0f
#define YAW_MANUAL_ANGLE_PID_KI			0.0f
#define YAW_MANUAL_ANGLE_PID_KD		0.0f
//速度环
#define YAW_MANUAL_SPEED_PID_KP			160.0f
#define YAW_MANUAL_SPEED_PID_KI				0.0f
#define YAW_MANUAL_SPEED_PID_KD			0.0f
/***********************************自瞄PID参数****************************************/
//自瞄模式pitch 角度环 PID参数

//自瞄模式yaw 角度环 PID参数

/**********************************能量机关PID参数*****************************************/
//射符模式pitch 角度环 PID参数
#define PITCH_STBUF_ANGLE_PID_KP		30.0f//20
#define PITCH_STBUF_ANGLE_PID_KI			0.0f
#define PITCH_STBUF_ANGLE_PID_KD		0.0f
//射符速度环
#define PITCH_STBUF_SPEED_PID_KP 		130.0f//150
#define PITCH_STBUF_SPEED_PID_KI				0.0f
#define PITCH_STBUF_SPEED_PID_KD			0.0f
//射符模式yaw 角度环 PID参数
#define YAW_STBUF_ANGLE_PID_KP			30.0f//20
#define YAW_STBUF_ANGLE_PID_KI				0.0f
#define YAW_STBUF_ANGLE_PID_KD			0.0f
//射符速度环
#define YAW_STBUF_SPEED_PID_KP				150.0f//160
#define YAW_STBUF_SPEED_PID_KI					0.0f
#define YAW_STBUF_SPEED_PID_KD				0.0f
/***************************************************************************/
/************************** gimbal parameter *****************************/
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO    	(8192.0f/360.0f)
/* Reduction ratio of pitch axis motor *//*pitch轴电机的减速比*/
#define PIT_DECELE_RATIO       					1.0f
/* Reduction ratio of yaw axis motor *//*yaw轴电机的减速比*/
#define YAW_DECELE_RATIO       				1.0f
/* the positive direction of pitch axis motor *//*pitch轴电机正向*/
#define PIT_MOTO_POSITIVE_DIR  		1.0f
/* the positive direction of yaw axis motor *//*yaw轴电机正方向*/
#define YAW_MOTO_POSITIVE_DIR  1.0f
/* the positive direction of tirgger motor *//*电机正方向*/
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
    GIMBAL_ZERO_FORCE = 0, 			//云台无力 0
    GIMBAL_INIT,           									//云台初始化(归中) 1
    GIMBAL_MANUAL_MODE,   	 	//云台手动控制 2
    GIMBAL_RELATIVE_MODE, 		 //云台地盘分离模式 3
    GIMBAL_TRACK_ARMOR,			 //云台辅助射击 4
    GIMBAL_SHOOT_BUFF,					 //云台射击符文 5
    GIMBAL_PATROL_MODE,    		//云台吊射基地 6
    GIMBAL_DEBUG_MODE, 				//云台调试模式 7
    GIMBAL_MOTIONLESS,					//遥控无输入 8
    GIMBAL_SHOOT_DOGE,					//击打小陀螺 9
    GIMBAL_LOADING_BULLET,	//装弹 10
    GIMBAL_TURN180,									//云台yaw扭180° 11
} gimbal_behaviour_e;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;//电机反馈
    const Encoder_process_t *gimbal_encoder_measure;//编码器反馈
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
    fp32 current_set;       //通过pid计算，设置current，传给give_current
    int16_t given_current;  //这个值直接传给电机

    extKalman_t Error_Kalman;//定义一个云台角度误差卡尔曼kalman指针
} Gimbal_Motor_t;

typedef struct
{
float yaw_angle;//当前角度
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
    const ext_game_robot_status_t *gimbal_status_measure;//底盘裁判系统功率读取
    Gimbal_Motor_t gimbal_yaw_motor;
    Gimbal_Motor_t gimbal_pitch_motor;

    gimbal_behaviour_e 	gimbal_behaviour;
    gimbal_behaviour_e	last_gimbal_behaviour;
    float pitch_angle_dynamic_ref;			//角度动态输入
    float yaw_angle_dynamic_ref;
    float pitch_angle_PcCtrl_ref;			//PC角度动态输入
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
