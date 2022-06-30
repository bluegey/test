#ifndef UI_TASK_H
#define UI_TASK_H

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "start_task.h"

#include "gimbal_task.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "judgement_info.h"
#include "protocol.h"
#include "Judge_task.h"
#include "CAN_Receive.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "Remote_Control.h"
//#include "BSP_USART_Init.h"

#define UI_ANGLE_TO_RAD   0.01745329251994329576923690768489f

typedef struct
{ 
    const ext_robot_hurt_t		  *shoot_hurt_point;
    const ext_game_robot_status_t *chassis_status_measure;//底盘裁判系统功率读取
    const RC_ctrl_t *ui_remote_ctrl;
    const Gimbal_Motor_t			 *ui_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
    const Gimbal_Motor_t			 *ui_pitch_motor;
    const motor_measure_t 		  *friction_motor_measure[2];
} ui_t;

void ui_task(void  *pvParameters);

#endif

