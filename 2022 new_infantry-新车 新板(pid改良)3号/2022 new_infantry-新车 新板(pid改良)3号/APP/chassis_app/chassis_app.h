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
    CHASSIS_RELAX          	= 0,//地盘无力模式
    CHASSIS_INIT           	= 1,
    CHASSIS_FOLLOW_GIMBAL  	= 2,//自动跟随云台
    CHASSIS_SEPARATE_GIMBAL   = 3,//云台分离
    CHASSIS_DODGE_MODE        = 4,//小陀螺
} chassis_mode_e; //底盘运行模式


typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    const Encoder_process_t *chassis_encoder_measure;
} Chassis_Motor_t;


typedef struct
{
    first_order_filter_type_t chassis_filter_set_vz;

    const Gimbal_Motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
    const Gimbal_Motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
    const ext_power_heat_data_t *chassis_power_measure;//底盘裁判系统功率读取
    const ext_game_robot_status_t *chassis_status_measure;//底盘裁判系统功率读取
    const ext_robot_hurt_t *chassis_hurt_type;
    const monitor_t *chassis_monitor_point;
    chassis_mode_e chassis_mode;               //底盘控制状态机
    Chassis_Motor_t motor_chassis[4];          //底盘电机数据

    PidTypeDef chassis_angle_pid;              //底盘跟随角度pid
    PidTypeDef motor_speed_pid[4];             //底盘电机速度pid
    PidTypeDef dodge_pid;					   //闪避模式角速度pid

    const Angular_Handle *Gimbal_angle_gyro_point;
    const RC_ctrl_t *chassis_rc_ctrl;
    float RC_X_ChassisSpeedRef;			//左右动态输入
    float RC_Y_ChassisSpeedRef;			//前后动态输入
    float RC_Z_ChassisSpeedRef;			//旋转动态输入

    first_order_filter_type_t chassis_cmd_slow_set_vz;

    float vx;
    float vy;
    float vz;
    int16_t        rotate_x_offset;
    int16_t        rotate_y_offset;
    float            wheel_spd_ref[4];
    float						wheel_spd_fdb[4];
    float           	chassis_relative_angle_set;
    float						Chassis_Gyro_Error;//相对YAW的中心分离机械角度

} chassis_move_t;

extern void mecanum_calc(float vx, float vy, float vz, float speed[], chassis_move_t *power_ctrl); //全向算法

//功率控制方案
void Chassis_Power_Limit(chassis_move_t *power_ctrl);
//超级电容控制方案
void Super_power_ctrl(chassis_move_t *power_ctrl);

#endif

