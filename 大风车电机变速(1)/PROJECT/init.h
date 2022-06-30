#ifndef _INIT_H
#define _INIT_H




#define SERVO_INIT 1500
#define SERVO_MAX  2500
#define SERVO_MIN  500

//#define C_1 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)							//八个传感器
//#define C_2 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)
//#define C_3 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)
//#define C_4 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)

#define S_1 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)		

#define STOP_SET GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)		//作用遥控器使能,同上  传感器

#define Reset_SET  GPIO_ResetBits(GPIOB,GPIO_Pin_5)	//作用初始化标志,同上  传感器
#define Reset_stop GPIO_SetBits(GPIOB,GPIO_Pin_5)	

#define Q_ON  GPIO_SetBits(GPIOD,GPIO_Pin_2)    //4
#define Q_OFF GPIO_ResetBits(GPIOD,GPIO_Pin_2)

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

enum
{
	stop = 0,
	foreward,
	reversal
};

#include <string.h>
#include "stm32f10x.h"
#include "misc.h" 
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "devicer_pwm.h"
#include "devicer_led.h"
#include "devicer_can.h"
#include "devicer_remote.h"
#include "devicer_usart.h"
#include "devicer_pid.h"
#include "devicer_adc.h"
void KernelTaskInit(void);

#endif
