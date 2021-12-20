#ifndef	_CONTROL_H_	
#define	_CONTROL_H_

#include "stm32f10x.h"


//#define SYSTEM_OFF()			GPIO_SetBits(GPIOG,GPIO_Pin_0)
//#define SYSTEM_ON()				GPIO_ResetBits(GPIOG,GPIO_Pin_0)

//#define MOTOR_OFF()				GPIO_ResetBits(GPIOF,GPIO_Pin_15)
//#define MOTOR_ON()				GPIO_SetBits(GPIOF,GPIO_Pin_15)

#define SYSTEM_OFF()			GPIO_SetBits(GPIOG,GPIO_Pin_10)
#define SYSTEM_ON()				GPIO_ResetBits(GPIOG,GPIO_Pin_10)

#define MOTOR_OFF()				GPIO_SetBits(GPIOG,GPIO_Pin_11)
#define MOTOR_ON()				GPIO_ResetBits(GPIOG,GPIO_Pin_11)

#define FINE_TUNING_OFF()	{GPIO_SetBits(GPIOG,GPIO_Pin_4);GPIO_SetBits(GPIOG,GPIO_Pin_7);}
#define FINE_TUNING_ON()	{GPIO_ResetBits(GPIOG,GPIO_Pin_4);GPIO_ResetBits(GPIOG,GPIO_Pin_7);}

#define LIMIT_IRQ_ENABLE()		{	EXTI_ClearITPendingBit(EXTI_Line11);\
																EXTI_ClearITPendingBit(EXTI_Line12);\
																EXTI_ClearITPendingBit(EXTI_Line13);\
																EXTI_ClearITPendingBit(EXTI_Line14);\
																NVIC_EnableIRQ(EXTI15_10_IRQn);}
#define LIMIT_IRQ_DISABLE()		NVIC_DisableIRQ(EXTI15_10_IRQn);

#define MOVE_F	1
#define MOVE_B	2
#define MOVE_L	3
#define MOVE_R	4


void Control_Init(void);
void Platform_initial_position(void);
void Platform_work_position(void);
void StepMotor_Move(uint8_t dir,uint16_t speed);

#endif
