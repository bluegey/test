#ifndef _SHOOT_TASK_H_
#define _SHOOT_TASK_H_
#include "start_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"

#include "CAN_Receive.h"
#include "gimbal_task.h"
#include "Remote_Control.h"
#include "pid.h"
#include "user_lib.h"
#include "arm_math.h"
//射击控制周期
#define SHOOT_CONTROL_TIME 3
#define SHOOT_TASK_INIT_TIME 115

//单环控制拨弹轮
#define TRIGGER_SINGLE_PID_KP 									20.0f
#define TRIGGER_SINGLE_PID_KI 									0.0f
#define TRIGGER_SINGLE_PID_KD 									0.0f
#define TRIGGER_SINGLE_PID_MAX_OUT 		10000.0f
#define TRIGGER_SINGLE_PID_MAX_IOUT 	1000.0f
//双环控制拨弹轮
#define TRIGGER_ANGLE_PID_KP 									150.0f
#define TRIGGER_ANGLE_PID_KI 										0.0f
#define TRIGGER_ANGLE_PID_KD 								 	0.0f
#define TRIGGER_ANGLE_PID_MAX_OUT 	  10000.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT  	0.0f
#define TRIGGER_SPEED_PID_KP 										15.0f
#define TRIGGER_SPEED_PID_KI 										0.0f
#define TRIGGER_SPEED_PID_KD 									0.0f
#define TRIGGER_SPEED_PID_MAX_OUT 			10000.0f
#define TRIGGER_SPEED_PID_MAX_IOUT 		0.0f
//单环控制摩擦轮
#define FRICTION_LEFT_PID_KP 										20.0f
#define FRICTION_LEFT_PID_KI 											0.0f
#define FRICTION_LEFT_PID_KD 										0.0f
#define FRICTION_RIGHT_PID_KP 									20.0f
#define FRICTION_RIGHT_PID_KI 									0.0f
#define FRICTION_RIGHT_PID_KD 									0.0f
#define FRICTION_PID_MAX_OUT  								16000.0f
#define FRICTION_PID_MAX_IOUT 								0.0f
//左右摩擦轮
#define FRICTION_LEFT	1
#define FRICTION_RIGHT  0
//电机rmp 变化成 旋转速度的比例
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.0012208521548041f //=360/8191/36
#define FULL_COUNT 36


typedef enum
{
    SHOOT_STOP         	=  1,//射击模式选择,默认不动
    SHOOT_NORMAL      	=  2,
    SINGLE_SHOOT  	  	=  3,//单发
    RUNNING_FIRE  	  	=  4,//连发
} trigger_mode_e;




typedef struct
{
    PidTypeDef trigger_motor_single_pid;

    PidTypeDef trigger_angle_bicyclo_pid; //双环控制 角度环
    PidTypeDef trigger_speed_bicyclo_pid;//双环控制 速度环

    PidTypeDef friction_motor_left_pid;
    PidTypeDef friction_motor_right_pid;
    Gimbal_Control_t 		*gimbal_Behaviour_r;
    const motor_measure_t 		  *trigger_motor_measure;
    const Encoder_process_t 	  *trigger_encoder_measure;
    const Encoder_process_t 	  *friction_encoder_measure[2];
    const motor_measure_t 		  *friction_motor_measure[2];
    const RC_ctrl_t 			  *shoot_rc_ctrl;
    const ext_power_heat_data_t   *shoot_heat_measure;
    const ext_shoot_data_t 		  *shoot_data_measure;
    const ext_game_robot_status_t *shoot_status_measure;
    const ext_robot_hurt_t		  *shoot_hurt_point;
    const monitor_t 			  *shoot_monitor_point;

    trigger_mode_e trigger_motor_mode;
    trigger_mode_e trigger_motor_last_mode;

    int32_t trigger_motor_ecd_count;
    fp32 trigger_motor_speed;
    fp32 trigger_motor_set_speed;
    fp32 trigger_motor_angle;
    fp32 trigger_motor_set_angle;
    fp32 friction_motor_speed;
    fp32 friction_motor_speed_set;
    fp32 trigger_give_current;
} Shoot_Motor_t;





void shoot_task(void *pvParameters);

#endif /*_SHOOT_TASK_H_*/
