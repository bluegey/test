/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       chassis.c/h
  * @brief      完成底盘控制任务。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */
#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "start_task.h"
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
#include "chassis_app.h"
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//左右的遥控器通道号码
#define CHASSIS_X_CHANNEL 0
//前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL 1
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 4
//遥控器波轮（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VZ_RC_SEN 0.05f

#define CHASSIS_ACCEL_Z_NUM 0.1666666667f
/***********************************************************************/
/* chassis maximum translation speed, unit is mm/s */
#define MAX_CHASSIS_VX_SPEED   		CHASSIS_KB_RC_MAX_SPEED
#define MAX_CHASSIS_VY_SPEED   		CHASSIS_KB_RC_MAX_SPEED
/* chassis maximum rotation speed, unit is degree/s */
#define MAX_CHASSIS_VR_SPEED   		CHASSIS_KB_RC_MAX_SPEED//NO Modify
/***********************************************************************/
/* normalized remote controller proportion */
#define RC_RESOLUTION     660.0f
/* remote mode chassis move speed limit */
#define CHASSIS_KB_RC_MAX_SPEED		8000.0f
/* back and forward speed (mm/s) */
#define CHASSIS_RC_MAX_SPEED_X  CHASSIS_KB_RC_MAX_SPEED
#define CHASSIS_RC_MOVE_RATIO_X 1.0f
/* left and right speed (mm/s) */
#define CHASSIS_RC_MAX_SPEED_Y  CHASSIS_KB_RC_MAX_SPEED
#define CHASSIS_RC_MOVE_RATIO_Y 1.0f
/***********************************************************************/
/* wheel track distance(mm) */ //左右轮距
#define WHEELTRACK             360
/* wheelbase distance(mm) *///前后轴距
#define WHEELBASE              400
/* the perimeter of wheel(mm) */
#define PERIMETER              478
/***********************************************************************/
/* math relevant */
/* radian coefficient */
#define RADIAN_COEF        57.3f
/*角度转化比例 */
#define ANGLE_TO_RAD   0.01745329251994329576923690768489f
/* chassis motor use 3508 */
/* the deceleration ratio of chassis motor */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
#define BUFFER_TIME    800
/***********************************************************************/
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 				8.0f
#define M3505_MOTOR_SPEED_PID_KI 					0.0f
#define M3505_MOTOR_SPEED_PID_KD 				0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT  			10000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 			1000.0f


//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 			8.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 				0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 			0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  		MAX_CHASSIS_VR_SPEED
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 		100.0f

//小陀螺PID
#define DODGE_PID_KP							5.0f
#define DODGE_PID_KI								0.0f
#define DODGE_PID_KD							0.0f
#define CHASSIS_DODGE_PID_MAX_OUT  				MAX_CHASSIS_VR_SPEED
#define CHASSIS_DODGE_PID_MAX_IOUT  			0.0f

extern void chassis_task(void *pvParameters);
#endif
