#ifndef __LED_H
#define __LED_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////
//引脚定义
/*******************************************************/
#define LED11  PCout(10)
#define LED22  PCout(11)
#define LED33  PCout(12)
#define LED44  PDout(2)
#define LED1_PIN                  GPIO_Pin_10
#define LED1_GPIO_PORT            GPIOC
#define LED1_GPIO_CLK             RCC_AHB1Periph_GPIOC


#define LED2_PIN                  GPIO_Pin_11
#define LED2_GPIO_PORT            GPIOC
#define LED2_GPIO_CLK             RCC_AHB1Periph_GPIOC

#define LED3_PIN                  GPIO_Pin_12
#define LED3_GPIO_PORT            GPIOC
#define LED3_GPIO_CLK             RCC_AHB1Periph_GPIOC

#define LED4_PIN                  GPIO_Pin_2
#define LED4_GPIO_PORT            GPIOD
#define LED4_GPIO_CLK             RCC_AHB1Periph_GPIOD

#define LASER_PIN                  GPIO_Pin_2
#define LASER_GPIO_PORT            GPIOA
#define LASER_GPIO_CLK             RCC_AHB1Periph_GPIOA

/************************************************************/


/** 控制LED灯亮灭的宏，
	* LED低电平亮，设置ON=0，OFF=1
	* 若LED高电平亮，把宏设置成ON=1 ，OFF=0 即可
	*/
#define ON  0
#define OFF 1

/* 带参宏，可以像内联函数一样使用 */
#define LED1(a)	if (a)	\
        GPIO_ResetBits(LED1_GPIO_PORT,LED1_PIN);\
    else		\
        GPIO_SetBits(LED1_GPIO_PORT,LED1_PIN)

#define LED2(a)	if (a)	\
        GPIO_ResetBits(LED2_GPIO_PORT,LED2_PIN);\
    else		\
        GPIO_SetBits(LED2_GPIO_PORT,LED2_PIN)

#define LED3(a)	if (a)	\
        GPIO_ResetBits(LED3_GPIO_PORT,LED3_PIN);\
    else		\
        GPIO_SetBits(LED3_GPIO_PORT,LED3_PIN)


#define LED4(a)	if (a)	\
        GPIO_ResetBits(LED4_GPIO_PORT,LED4_PIN);\
    else		\
        GPIO_SetBits(LED4_GPIO_PORT,LED4_PIN)

#define LAESR(a)	if (a)	\
        GPIO_ResetBits(LASER_GPIO_PORT,LASER_PIN);\
    else		\
        GPIO_SetBits(LASER_GPIO_PORT,LASER_PIN)

#define LED1_TOGGLE()			GPIO_ToggleBits(LED1_GPIO_PORT,LED1_PIN)
#define LED2_TOGGLE()			GPIO_ToggleBits(LED2_GPIO_PORT,LED2_PIN)
#define LED3_TOGGLE()			GPIO_ToggleBits(LED3_GPIO_PORT,LED3_PIN)
#define LED4_TOGGLE()			GPIO_ToggleBits(LED4_GPIO_PORT,LED4_PIN)

#define LED_ALLOFF	\
    LED1(OFF);\
    LED2(OFF);\
    LED3(OFF);\
    LED4(OFF)

#define LED_ALLTOGGLE	\
    LED1_TOGGLE();\
    LED2_TOGGLE();\
    LED3_TOGGLE();\
    LED4_TOGGLE()

#define LED_ALLON	\
    LED1(ON);\
    LED2(ON);\
    LED3(ON);\
    LED4(ON)

// 流水灯
#define DETECT_FLOW_LED_ON(num)	 		GPIO_ResetBits(GPIOC, GPIO_Pin_12 >> (num));
#define DETECT_FLOW_LED_OFF(num)	 	GPIO_SetBits(GPIOC, GPIO_Pin_12 >> (num));
void LED_Init(void);//初始化
void LASER_Init(void);


#endif
