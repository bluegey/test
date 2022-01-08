//#include "Exit.h"
//#include "oled.h"
//#include "OLED_Task.h"
//#include "stm32f10x.h"                  // Device header
//#include "stm32f10x_exti.h"
//#include "key.h"
//#include "delay.h"
//#include "led.h"
//extern OLED_OptionDisplay ModeOption ;
//extern Motor_ModeOption  MotorMode; 
//extern Motor_SetOption   MotorSet;
//void Exit_Init()
//{
//	EXTI_InitTypeDef EXTI_InitStructer;
//	NVIC_InitTypeDef NVIC_InitStructer;

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

//	EXTI_InitStructer.EXTI_Line=ADVANCE_EXTI_Linex;
//	EXTI_InitStructer.EXTI_Mode=EXTI_Mode_Interrupt;
//	EXTI_InitStructer.EXTI_Trigger=EXTI_Trigger_Falling;
//	EXTI_InitStructer.EXTI_LineCmd=ENABLE;

//	GPIO_EXTILineConfig(ADVANCE_EXTI_GPIO,ADVANCE_EXTI_PinSource);

//	EXTI_Init(&EXTI_InitStructer);

//	NVIC_InitStructer.NVIC_IRQChannel=EXTI3_IRQn;
//	NVIC_InitStructer.NVIC_IRQChannelPreemptionPriority=0;
//	NVIC_InitStructer.NVIC_IRQChannelSubPriority=2;
//	NVIC_InitStructer.NVIC_IRQChannelCmd=ENABLE;

//	NVIC_Init(&NVIC_InitStructer);	
//}
//void EXTI3_IRQHandler(void)
//{
////delay_ms(10);
//if(KEY_Scan(ADVANCE_KEY_GPIOx,ADVANCE_GPIOx_KEY0_Pinx) ==KEY_ON)
//{
//LED0=~LED0;	
//}
//}

