#include "led.h"
/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       led.c/h
  * @brief
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020     		RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
**/

//LED IO��ʼ��
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd ( LED1_GPIO_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd ( LED4_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_2;	//LED0��LED1��ӦIO��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);					//��ʼ��GPIO

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//����
	GPIO_Init(GPIOD, &GPIO_InitStructure);					//��ʼ��GPIO
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



