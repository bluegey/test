#include "Control.h"
#include "mydelay.h"

//初始化控制IO口
void Control_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG| RCC_APB2Periph_AFIO, ENABLE);
	
	//总开关和切割电机
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	//GPIO_Pin_0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	SYSTEM_OFF();
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//	GPIO_Pin_15
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	MOTOR_OFF();
	
	//控制平台
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	FINE_TUNING_OFF();
	GPIO_SetBits(GPIOG,GPIO_Pin_2);
	GPIO_SetBits(GPIOG,GPIO_Pin_3);
	GPIO_SetBits(GPIOG,GPIO_Pin_5);
	GPIO_SetBits(GPIOG,GPIO_Pin_6);
	
	//限位输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 ;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	LIMIT_IRQ_DISABLE();
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource11);
	EXTI_InitStructure.EXTI_Line=EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource12);
	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
	EXTI_Init(&EXTI_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource13);
	EXTI_InitStructure.EXTI_Line=EXTI_Line13;
	EXTI_Init(&EXTI_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource14);
	EXTI_InitStructure.EXTI_Line=EXTI_Line14;
	EXTI_Init(&EXTI_InitStructure);
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_DeInit(TIM3);

	TIM_TimeBaseStructure.TIM_Period = 1000-1;       //当定时器从0计数到999，即为1000次，为一个定时周期
	TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //设置预分频：不预分频，即为18MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数：4分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_Pulse = 1000;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);	 //使能通道1
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);			 // 使能TIM3重载寄存器ARR
	
	TIM_Cmd(TIM3, ENABLE);                   //使能定时器3
}


//初始化平台到安全位置
u8 flag=0;
void Platform_initial_position(void)
{
	uint16_t i=1500;
	while(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_12)==0)
	{
	StepMotor_Move(MOVE_R,i/10);
		if(i>500)
			i--;

	}
	i=1200;
	while(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_11)==0)
	{

		StepMotor_Move(MOVE_B,i/10);
		if(i>300)
			i--;}

}

void Platform_work_position(void)
{
	uint16_t i,j;
	j=1200;
	for(i=0;i<36200;i++)
	{
		StepMotor_Move(MOVE_F,j/10);
		StepMotor_Move(MOVE_F,j/10);
		if(j>300)
			j--;
	}
	MOTOR_ON();
	for(i=0;i<5850;i++)
	{
		StepMotor_Move(MOVE_L,300);
	}
}

void StepMotor_Move(uint8_t dir,uint16_t speed)
{
	if(dir==MOVE_F)
	{
		my_delayus(speed);
		GPIO_ResetBits(GPIOG,GPIO_Pin_2);
		my_delayus(speed);
		GPIO_SetBits(GPIOG,GPIO_Pin_2);
	}
	else if(dir==MOVE_B)
	{
		my_delayus(speed);
		GPIO_ResetBits(GPIOG,GPIO_Pin_3);
		my_delayus(speed);
		GPIO_SetBits(GPIOG,GPIO_Pin_3);
	}
	else if(dir==MOVE_L)
	{
		my_delayus(speed);
		GPIO_ResetBits(GPIOG,GPIO_Pin_5);
		my_delayus(speed);
		GPIO_SetBits(GPIOG,GPIO_Pin_5);
	}
	else if(dir==MOVE_R)
	{
		my_delayus(speed);
		GPIO_ResetBits(GPIOG,GPIO_Pin_6);
		my_delayus(speed);
		GPIO_SetBits(GPIOG,GPIO_Pin_6);
	}
}

void EXTI15_10_IRQHandler(void)
{
	if ( EXTI_GetITStatus(EXTI_Line11) != RESET )
	{
		SYSTEM_OFF();
		EXTI_ClearITPendingBit(EXTI_Line11);
	}	
	if ( EXTI_GetITStatus(EXTI_Line12) != RESET )
	{
		SYSTEM_OFF();
		EXTI_ClearITPendingBit(EXTI_Line12);
	}	
	if ( EXTI_GetITStatus(EXTI_Line13) != RESET )
	{
		SYSTEM_OFF();
		EXTI_ClearITPendingBit(EXTI_Line13);
	}	
	if ( EXTI_GetITStatus(EXTI_Line14) != RESET )
	{
		SYSTEM_OFF();
		EXTI_ClearITPendingBit(EXTI_Line14);
	}		
}
