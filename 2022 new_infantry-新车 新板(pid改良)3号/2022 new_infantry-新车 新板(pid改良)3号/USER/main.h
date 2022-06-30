#ifndef _MAIN_H_
#define _MAIN_H_


/*
*********************************************************************************************************
* SYSTEM
*********************************************************************************************************
*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
/*
*********************************************************************************************************
* ???
*********************************************************************************************************
*/
#include "stm32f4xx.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
/*
*********************************************************************************************************
* OS
*********************************************************************************************************
*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"
/*
*********************************************************************************************************
* Math
*********************************************************************************************************
*/
#include "pid.h"
#include "arm_math.h"
#include "user_lib.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "Ramp_Control.h"
/*
*********************************************************************************************************
* HardWare
*********************************************************************************************************
*/
#include "led.h"
#include "rc.h"
#include "can.h"
#include "timer.h"
/*
*********************************************************************************************************
* Task
*********************************************************************************************************
*/
#include "start_task.h"
#include "Remote_Control.h"
#include "CAN_Receive.h"
#include "IMUTask.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "shoot_task.h"
#include "Tx2_task.h"
#include "detect_task.h"
#include "Judge_Task.h"
#include "get_judge_measure.h"
#include "ui_task.h"
#include "oled.h"
#include "beep.h"
#include "adc.h"
void BSP_Init(void);

#endif

