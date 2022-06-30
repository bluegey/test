#ifndef GIMBAL_APP_H
#define GIMBAL_APP_H
#include "stm32f4xx.h"
#include "sys.h"
#include "string.h"
#include "stdio.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "arm_math.h"
#include "user_lib.h"
#include "can.h"
#include "CAN_receive.h"
#include "pid.h"
#include "math.h"


/***********undefine****头文件重复********/


typedef enum
{
	NORMAL=0,
	I_OUT=1,   //积分分离
	RAPEZOID=2,//梯形
	TIME_PID=3,//一种引入时间的pid
	FUTURE_PID,//可预测型PID
}PID_mode_t;
typedef struct
{
	
    fp32 kp;
    fp32 ki;
    fp32 kd;
    fp32 err_limit;
	  fp32 err_dead;
    fp32 set;
    fp32 get;
    fp32 err[3]; //误差项 0最新 1上一次 2上上次
    fp32 err_sum;//用于积分计算
	  
    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
	  PID_mode_t pid_mode;
	
} Gimbal_PID_t;


typedef struct
{
    const motor_measure_t *gimbal_motor_measure;//电机反馈
    const Encoder_process_t *gimbal_encoder_measure;//编码器反馈
    PidTypeDef gimbal_motor_gyro_pid;
	  Gimbal_PID_t gimbal_motor_angle_pid;

	
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


extern void gimbal_pid_reset(Gimbal_PID_t *pid, float p, float i, float d);
extern void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);//云台PID初始化
extern fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, extKalman_t Gim); //PID解算
extern void GIMBAL_PIDpro_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, extKalman_t Gim); //PID积分分离式解算
extern void GIMBAL_PID_Dfuture_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, extKalman_t Gim); //PID预测解算
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);//得到相对角度
extern void gimbal_relative_angle_control(Gimbal_Motor_t *gimbal_motor);//云台相对角度控制
extern void gimbal_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);//云台绝对角度控制


#endif

