#ifndef   __OLED_TASK_H
#define   __OLED_TASK_H
#include "judge_Task.h"
#include "detect_task.h"
#include "pid.h"
#include "can.h"
#include "remote_control.h"
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"
#include "IMUTask.h"
#include "oled.h"
#include "rc.h"
#include "led.h"
#include "timer.h"
#include "beep.h"
#include "adc.h"

typedef struct
{
const Angular_Handle *gimbal_angle_gyro_point;
const RC_ctrl_t *GET_rc_point;
const monitor_t *gimbal_monitor_point;
int32_t oled_time; //获取定时器时间
	int mid_time;
	fp32 Anglex,Angley,Anglez;
fp32 Gyrox,Gyroy,Gyroz;
fp32 ch1,ch2,ch3,ch4;
	fp32 oled_VOLTAGE;
	fp32 oled_voltage;
	
}
oled_control_t;

static void oled_init(oled_control_t *oled_init);
static void oled_Feedback_Update(oled_control_t *oled_feedback);
static void oled_show(oled_control_t *oled_show);
static void oled_running(oled_control_t *oled_go);
static void beep_detect(int time);
int myabs(int a);
#endif
