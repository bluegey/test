#include "Init.h"

u16 SuperCapacitor_CompareState;//比较器状态
u16 Output_ratio = 0;//输出占空比

void BSP_SuperCapacitor_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	Battery_Enable;
	Charge_Disable;
}

//void TIM3_PWM_Init(u16 arr,u16 psc)
//{

//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef		TIM_OCInitStructure;

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);			//SHI NEN TIM3
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);			//SHI NEN GPIOC
//	
//	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB,&GPIO_InitStructure);
//	
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_Period=arr;
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
//		
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
//	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);

//	TIM_Cmd(TIM3,ENABLE);	
//	
//  TIM_SetCompare2(TIM3,Output_ratio);
//}

