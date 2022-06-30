#include "devicer_pwm.h"

#define pwm_mosfet_off	10000

int Motor_Pwm=0;              //���PWM
int Motor_pwm=0;              //���PWM
int t = 0;

void  Motor_Init(void)
{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);  //����GPIOBʱ��
//    GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);

//    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4|GPIO_Pin_5;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//    GPIO_Init(GPIOA,&GPIO_InitStructure);

//    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_10|GPIO_Pin_11;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//    GPIO_Init(GPIOB,&GPIO_InitStructure);

//    GPIO_ResetBits(GPIOA,GPIO_Pin_4);//��ָ��λ�õ� PA4		EN4
//    GPIO_SetBits(GPIOA,GPIO_Pin_5);	//��ָ��λ�ø� PA5			EN3

//    GPIO_ResetBits(GPIOB,GPIO_Pin_11);//��ָ��λ�õ� PB11		EN2
//    GPIO_SetBits(GPIOB,GPIO_Pin_10);	//��ָ��λ�ø� PB10		EN1
////		GPIO_SetBits(GPIOA,GPIO_Pin_6);
////		GPIO_ResetBits(GPIOA,GPIO_Pin_7);
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);  //����GPIOBʱ��
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);

    GPIO_ResetBits(GPIOA,GPIO_Pin_4);//��ָ��λ�õ� PA4		EN4
    GPIO_SetBits(GPIOA,GPIO_Pin_5);	//��ָ��λ�ø� PA5			EN3

    GPIO_ResetBits(GPIOC,GPIO_Pin_5);//��ָ��λ�õ� PC5		EN2
    GPIO_SetBits(GPIOC,GPIO_Pin_4);	//��ָ��λ�ø� PB4		EN1




}



//TIM3 PWM���ֳ�ʼ��
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
//Tout= ((arr+1)*(psc+1))/Tclk��
void TIM3_PWM_Init(u16 arr,u16 psc)
{
      TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx		
}
/*Ƶ�ʣ� 36M/((arr+1)*(psc+1))*/
void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
		 t++;
		}
}
void TIM2_PWM_Init(u16 hz)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    u16 PrescalerValue = 0;
    u32 hz_set = 10000*hz;
    
    hz_set = LIMIT (hz_set,1,36000000);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);

    //���ø�����Ϊ�����������,���TIM2 CH1��PWM���岨��	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; 			//TIM_CH1/2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	//�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_DeInit(TIM2);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
    
    PrescalerValue = (uint16_t) ( ( SystemCoreClock/2  ) / hz_set ) - 1;
    //��ʼ��TIM2
    TIM_TimeBaseStructure.TIM_Period = 9999; 						//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler =PrescalerValue; 						//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 					//����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	//TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    //��ʼ��TIM2 Channel1 PWMģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_Pulse=SERVO_INIT;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//�������:TIM����Ƚϼ��Ը�
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_Pulse= SERVO_INIT;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//�������:TIM����Ƚϼ��Ը�
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);


    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  				//ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  				//ʹ��TIM2��CCR2�ϵ�Ԥװ�ؼĴ���
    
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_Cmd(TIM2, ENABLE);  
//    Motor_Init();
}
//TIM4 PWM���ֳ�ʼ��
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
//Tout= ((arr+1)*(psc+1))/Tclk��
void TIM4_PWM_Init(u16 arr,u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    //���ø�����Ϊ�����������,���TIM4 CH1��PWM���岨��	GPIOB.6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9; 			//TIM_CH1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	//�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);



    //��ʼ��TIM2
    TIM_TimeBaseStructure.TIM_Period = arr; 						//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler =psc; 						//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 					//����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	//TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    //��ʼ��TIM2 Channel1 PWMģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//�������:TIM����Ƚϼ��Ը�
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//�������:TIM����Ƚϼ��Ը�
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);


    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  				//ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���

    TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
//    Motor_Init();
}

void PWM_GPIO_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PA�˿�ʱ��
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 //LED0-->PB.5 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
// GPIO_ResetBits(GPIOA,GPIO_Pin_5);						 //PA.5 �����

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	    		 //LED1-->PE.5 �˿�����, �������
 GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
// GPIO_ResetBits(GPIOA,GPIO_Pin_4); 						 //PA.4 ����� 
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 //LED1-->PE.5 �˿�����, �������
 GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
// GPIO_ResetBits(GPIOA,GPIO_Pin_6); 						 //PA.4 ����� 
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	    		 //LED1-->PE.5 �˿�����, �������
 GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
// GPIO_ResetBits(GPIOA,GPIO_Pin_7); 						 //PA.4 ����� 
}




/*********************************************************************
*��������:	Change_PWM
*��������:	�ı�ָ�����PWMռ�ձ�
*�������:
*�������:
*��ע:
*********************************************************************/
void Change_Motor_PWM(uint8_t num,u16 duty)
{

    switch(num)
    {
    case 0: {
        TIM_SetCompare1(TIM3,0);
        TIM_SetCompare2(TIM3,0);
    }
    break;
    case 2: {

        TIM_SetCompare1(TIM3,duty);  //PA6
        TIM_SetCompare2(TIM3,0);     //PA7
				GPIO_ResetBits(GPIOA,GPIO_Pin_5);//��ָ��λ�õ� PA4
				GPIO_SetBits(GPIOA,GPIO_Pin_4);	//��ָ��λ�ø� PA5

    }
    break;
    case 1: {
				
        TIM_SetCompare2(TIM3,duty);
        TIM_SetCompare1(TIM3,0);
			  GPIO_ResetBits(GPIOA,GPIO_Pin_4);//��ָ��λ�õ� PA4
				GPIO_SetBits(GPIOA,GPIO_Pin_5);	//��ָ��λ�ø� PA5

    }
    break;

    default:
        break;
    }
}

void Change_Motor_pwm(uint8_t num,u16 duty)
{

    switch(num)
    {
    case 0: {
        TIM_SetCompare3(TIM3,0);
        TIM_SetCompare4(TIM3,0);

    }
    break;
    case 2: {
        TIM_SetCompare3(TIM3,0);
        TIM_SetCompare4(TIM3,duty);
    }
    break;
    case 1: {
        TIM_SetCompare3(TIM3,duty);
        TIM_SetCompare4(TIM3,0);
    }
    break;

    default:
        break;
    }
}




//�����ת
void Device_Corotation(void)
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);//��ָ��λ�õ� PB4
    GPIO_SetBits(GPIOA,GPIO_Pin_5);	//��ָ��λ�ø� PB5
}
void Device_corotation(void)
{
    GPIO_ResetBits(GPIOC,GPIO_Pin_5);//��ָ��λ�õ�
    GPIO_SetBits(GPIOC,GPIO_Pin_4);	//��ָ��λ�ø� 
}


//�����ת
void Device_Rollback(void)
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_5);//��ָ��λ�õ� PB5
    GPIO_SetBits(GPIOA,GPIO_Pin_4);	//��ָ��λ�ø� PB4
}
void Device_rollback(void)
{
    GPIO_SetBits(GPIOC,GPIO_Pin_5);
    GPIO_ResetBits(GPIOC,GPIO_Pin_4);
}

//���ֹͣ
void Device_Stop(void)
{
    GPIOA->ODR|=(3<<4);//��ָ��λ�ø�  PB4 PB5

}
void Device_stop(void)
{
    GPIO_SetBits(GPIOC,GPIO_Pin_5);
    GPIO_SetBits(GPIOC,GPIO_Pin_4);
}


/*********************************************************************
*��������:	Change_RotatePWM
*��������:	�ı�ָ�����PWM��ת��
*�������:
*�������:
*��ע:
*********************************************************************/
void Change_RotatePWM(void)
{

    if(Motor_Pwm>0)
    {
        Change_Motor_PWM(1,absolute_value(Motor_Pwm));//��ת
        Device_Corotation();
    }
    else if(Motor_Pwm<0)
    {
        Change_Motor_PWM(2,absolute_value(Motor_Pwm));//��ת
        Device_Rollback();
    }
    else
    {
        Change_Motor_PWM(0,0);
        Device_Stop();
    }
}

void Change_Rotatepwm(void)
{

    if(Motor_pwm>0)
    {
        Change_Motor_pwm(1,absolute_value(Motor_pwm));
        Device_corotation();
    }
    else if(Motor_pwm<0)
    {
        Change_Motor_pwm(2,absolute_value(Motor_pwm));
        Device_rollback();
    }
    else
    {
        Change_Motor_pwm(0,0);
        Device_stop();
    }
}


//26 27 28 29
void PWM_SET_INIT(int PWM_SET)
{
}

