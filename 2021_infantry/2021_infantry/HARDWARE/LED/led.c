#include "led.h"
/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       led.c/h
  * @brief
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020     		RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
**/

//LED IO初始化
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd ( LED1_GPIO_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd ( LED4_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED4_PIN | LED2_PIN | LED3_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED1_PIN;
    GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);

    LED_ALLOFF;
}

void LASER_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd ( LASER_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin 	= LASER_PIN;
    GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;

    GPIO_Init(LASER_GPIO_PORT, &GPIO_InitStructure);

    LAESR(ON);
}



