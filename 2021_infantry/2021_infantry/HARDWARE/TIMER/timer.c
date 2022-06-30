 /**
   ****************************(C) COPYRIGHT 2020 NCIST****************************
   * @file       timer.c
   * @brief      
   *             		TIM5 PWM���ֳ�ʼ��
   *             		PWM�����ʼ��
   *             
   * @note       
   * @history
   *  Version    Date            Author          Modification
   *  V1.0.0     2021-07-17     	   RM              1. ���
   *
   @verbatim
   ==============================================================================

   ==============================================================================
   @endverbatim
   ****************************(C) COPYRIGHT 2020 NCIST****************************
   */
#include "timer.h"

void TIM5_PWM_Init(u32 arr, u32 psc)//arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  	//TIM5ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5); //GPIOA0����Ϊ��ʱ��5

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;           //GPIOA0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);             //��ʼ��PA0

    TIM_TimeBaseStructure.TIM_Prescaler = psc; //��ʱ����Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = arr; //�Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //��ʼ����ʱ��5

    //��ʼ��TIM2 Channel PWMģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ��TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ�ʹ�����
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //������ԣ�TIM����Ƚϼ��Ե�
    TIM_OCInitStructure.TIM_Pulse = 0;//�Ƚϳ�ʼֵ
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC1

    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ǵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM5, ENABLE); //ARPEʹ��

    TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM5
}
