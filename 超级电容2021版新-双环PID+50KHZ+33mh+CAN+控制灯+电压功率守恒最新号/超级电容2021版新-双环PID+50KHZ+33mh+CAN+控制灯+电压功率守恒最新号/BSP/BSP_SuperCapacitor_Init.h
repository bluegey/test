#ifndef __BSP_SUPERCAPACITOR_INIT_H_
#define __BSP_SUPERCAPACITOR_INIT_H_

#include "stm32f10x.h"


#define Battery_Enable	 GPIO_ResetBits(GPIOB,GPIO_Pin_7)//��ؿ�
#define Battery_Disable  GPIO_SetBits(GPIOB,GPIO_Pin_7)//��ع�


#define SuperCapacitor_Enable  GPIO_SetBits(GPIOB,GPIO_Pin_6)//�������ݿ�
#define SuperCapacitor_Disable GPIO_ResetBits(GPIOB,GPIO_Pin_6)//�������ݹ�

#define Charge_Enable   GPIO_SetBits(GPIOA,GPIO_Pin_7)//�������ݳ��
#define Charge_Disable  GPIO_ResetBits(GPIOA,GPIO_Pin_7)


//#define Battery_Enable	 GPIO_ResetBits(GPIOA,GPIO_Pin_7)//��ؿ�
//#define Battery_Disable  GPIO_SetBits(GPIOA,GPIO_Pin_7)//��ع�


//#define SuperCapacitor_Enable  GPIO_SetBits(GPIOB,GPIO_Pin_6)//�������ݿ�
//#define SuperCapacitor_Disable GPIO_ResetBits(GPIOB,GPIO_Pin_6)//�������ݹ�

//#define Charge_Enable   GPIO_SetBits(GPIOB,GPIO_Pin_7)//�������ݳ��
//#define Charge_Disable  GPIO_ResetBits(GPIOB,GPIO_Pin_7)

extern u16 SuperCapacitor_CompareState;
extern u16 Output_ratio;

void BSP_SuperCapacitor_Init(void);
//void TIM3_PWM_Init(u16 arr,u16 psc);






#endif

