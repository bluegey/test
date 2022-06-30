#ifndef _BSP_GPIO_INIT_H_
#define _BSP_GPIO_INIT_H_


#include "stm32f10x.h"                  // Device header


#define LED0_ON   GPIO_SetBits(GPIOB,GPIO_Pin_4)
#define LED0_OFF  GPIO_ResetBits(GPIOB,GPIO_Pin_4)
#define LED1_ON     GPIO_SetBits(GPIOB,GPIO_Pin_3)
#define LED1_OFF    GPIO_ResetBits(GPIOB,GPIO_Pin_3)


void BSP_GPIO_Init(void);
void LED0_Runsign(unsigned times);
void LED1_Runsign(void);
void TIM5_Int_Init(u16 arr,u16 psc);
void TIM2_PWM_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
#endif

