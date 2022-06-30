#ifndef _INIT_H_
#define _INIT_H_

#include "cmsis_os.h"
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_exti.h"

#include "String.h"
#include "mydelay.h"
#include "judgement_info.h"
#include "protocol.h"

#include "BSP_GPIO_Init.h"
#include "BSP_SuperCapacitor_Init.h"
#include "BSP_USART_Init.h"
#include "BSP_CAN_Init.h"
#include "BSP_IIC_Init.h"
#include "INA260_Driver.h"
#include "mpu6050_interrupt.h"
#include "mpu6050.h"

#include "CAN_Task.h"
#include "Judge_Task.h"
#include "Monitor_Task.h"
#include "PowerSupply_Task.h"
#include "BSP_ADC_Init.h"
#include "PID_Control.h"
void KernelTaskInit(void);
void NVIC_StateInit(void );
void Current_PID(int MAXCurrent,float MAXVoltage);
extern osSemaphoreId_t GRYO_Semaphore;
void Power_Mod_Select(uint8_t inpower,uint8_t flag);
void current_select(float current,float power);
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

#define RED 0
#define BLUE 1

#endif




