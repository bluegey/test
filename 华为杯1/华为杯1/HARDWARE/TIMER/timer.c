#include "timer.h"

void TIM5_PWM_Init(u32 arr, u32 psc)//arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  	//TIM5ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5); //GPIOA0����Ϊ��ʱ��5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); //GPIOA0����Ϊ��ʱ��5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5); //GPIOA0����Ϊ��ʱ��5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5); //GPIOA0����Ϊ��ʱ��5

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;           //GPIOA0
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
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //������ԣ�TIM����Ƚϼ��Ե�
    TIM_OCInitStructure.TIM_Pulse = 0;//�Ƚϳ�ʼֵ
		
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC1
		TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC1
		TIM_OC3Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC1
		TIM_OC4Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC1

    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ǵ�Ԥװ�ؼĴ���
		TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ǵ�Ԥװ�ؼĴ���
		TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ǵ�Ԥװ�ؼĴ���
		TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ǵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM5, ENABLE); //ARPEʹ��

    TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM5
}

void TIM3_PWM_Init(u32 arr, u32 psc)
{
		GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  	//TIM5ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3); //GPIOA0����Ϊ��ʱ��5
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); //GPIOA0����Ϊ��ʱ��5

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;           //GPIOA0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);             //��ʼ��PA0
	
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           //GPIOA0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);             //��ʼ��PA0

    TIM_TimeBaseStructure.TIM_Prescaler = psc; //��ʱ����Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = arr; //�Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //��ʼ����ʱ��5

    //��ʼ��TIM2 Channel PWMģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ��TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ�ʹ�����
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //������ԣ�TIM����Ƚϼ��Ե�
    TIM_OCInitStructure.TIM_Pulse = 0;//�Ƚϳ�ʼֵ
		
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC1
		TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC1

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ǵ�Ԥװ�ؼĴ���
		TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ǵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM3, ENABLE); //ARPEʹ��

    TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM5
}

void TIM4_PWM_Init(u32 arr, u32 psc)
{
		GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  	//TIM5ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTAʱ��

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4); //GPIOA0����Ϊ��ʱ��5

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           //GPIOA0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOB, &GPIO_InitStructure);             //��ʼ��PA0

    TIM_TimeBaseStructure.TIM_Prescaler = psc; //��ʱ����Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = arr; //�Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //��ʼ����ʱ��5

    //��ʼ��TIM2 Channel PWMģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ��TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ�ʹ�����
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //������ԣ�TIM����Ƚϼ��Ե�
    TIM_OCInitStructure.TIM_Pulse = 0;//�Ƚϳ�ʼֵ
		
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC1
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC1

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ǵ�Ԥװ�ؼĴ���
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ǵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM4, ENABLE); //ARPEʹ��

    TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM5
}

void TIM1_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;/*��ʱ��1���ж�ͨ��ʹ��*/
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

TIM_ClearFlag(TIM1,TIM_FLAG_Update);/*����±�־λ*/
TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);/*ʹ���ж�*/

	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIMx					 
}
//��ʱ��3�жϷ������
u32 time1_cntr;
void TIM1_UP_TIM10_IRQHandler(void)   //TIM3�ж�
{
//	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
//		{
		time1_cntr++;
		TIM_ClearFlag(TIM1,TIM_FLAG_Update);/*���жϱ�־*/
//		}
}
