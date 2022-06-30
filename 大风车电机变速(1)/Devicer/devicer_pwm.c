#include "devicer_pwm.h"

#define pwm_mosfet_off	10000

int Motor_Pwm=0;              //电机PWM
int Motor_pwm=0;              //电机PWM
int t = 0;

void  Motor_Init(void)
{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);  //开启GPIOB时钟
//    GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);

//    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4|GPIO_Pin_5;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//    GPIO_Init(GPIOA,&GPIO_InitStructure);

//    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_10|GPIO_Pin_11;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//    GPIO_Init(GPIOB,&GPIO_InitStructure);

//    GPIO_ResetBits(GPIOA,GPIO_Pin_4);//将指定位置低 PA4		EN4
//    GPIO_SetBits(GPIOA,GPIO_Pin_5);	//将指定位置高 PA5			EN3

//    GPIO_ResetBits(GPIOB,GPIO_Pin_11);//将指定位置低 PB11		EN2
//    GPIO_SetBits(GPIOB,GPIO_Pin_10);	//将指定位置高 PB10		EN1
////		GPIO_SetBits(GPIOA,GPIO_Pin_6);
////		GPIO_ResetBits(GPIOA,GPIO_Pin_7);
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);  //开启GPIOB时钟
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);

    GPIO_ResetBits(GPIOA,GPIO_Pin_4);//将指定位置低 PA4		EN4
    GPIO_SetBits(GPIOA,GPIO_Pin_5);	//将指定位置高 PA5			EN3

    GPIO_ResetBits(GPIOC,GPIO_Pin_5);//将指定位置低 PC5		EN2
    GPIO_SetBits(GPIOC,GPIO_Pin_4);	//将指定位置高 PB4		EN1




}



//TIM3 PWM部分初始化
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
//Tout= ((arr+1)*(psc+1))/Tclk；
void TIM3_PWM_Init(u16 arr,u16 psc)
{
      TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM3, ENABLE);  //使能TIMx		
}
/*频率： 36M/((arr+1)*(psc+1))*/
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志 
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

    //设置该引脚为复用输出功能,输出TIM2 CH1的PWM脉冲波形	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; 			//TIM_CH1/2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	//复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_DeInit(TIM2);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
    
    PrescalerValue = (uint16_t) ( ( SystemCoreClock/2  ) / hz_set ) - 1;
    //初始化TIM2
    TIM_TimeBaseStructure.TIM_Period = 9999; 						//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler =PrescalerValue; 						//设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 					//设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	//TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    //初始化TIM2 Channel1 PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//比较输出使能
    TIM_OCInitStructure.TIM_Pulse=SERVO_INIT;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//比较输出使能
    TIM_OCInitStructure.TIM_Pulse= SERVO_INIT;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//输出极性:TIM输出比较极性高
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);


    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  				//使能TIM2在CCR1上的预装载寄存器
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  				//使能TIM2在CCR2上的预装载寄存器
    
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_Cmd(TIM2, ENABLE);  
//    Motor_Init();
}
//TIM4 PWM部分初始化
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
//Tout= ((arr+1)*(psc+1))/Tclk；
void TIM4_PWM_Init(u16 arr,u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    //设置该引脚为复用输出功能,输出TIM4 CH1的PWM脉冲波形	GPIOB.6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9; 			//TIM_CH1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	//复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);



    //初始化TIM2
    TIM_TimeBaseStructure.TIM_Period = arr; 						//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler =psc; 						//设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 					//设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	//TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    //初始化TIM2 Channel1 PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//输出极性:TIM输出比较极性高
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//输出极性:TIM输出比较极性高
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);


    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  				//使能TIM4在CCR1上的预装载寄存器

    TIM_Cmd(TIM4, ENABLE);  //使能TIM4
//    Motor_Init();
}

void PWM_GPIO_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PA端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 //LED0-->PB.5 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
// GPIO_ResetBits(GPIOA,GPIO_Pin_5);						 //PA.5 输出低

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	    		 //LED1-->PE.5 端口配置, 推挽输出
 GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
// GPIO_ResetBits(GPIOA,GPIO_Pin_4); 						 //PA.4 输出低 
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 //LED1-->PE.5 端口配置, 推挽输出
 GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
// GPIO_ResetBits(GPIOA,GPIO_Pin_6); 						 //PA.4 输出低 
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	    		 //LED1-->PE.5 端口配置, 推挽输出
 GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
// GPIO_ResetBits(GPIOA,GPIO_Pin_7); 						 //PA.4 输出低 
}




/*********************************************************************
*函数名称:	Change_PWM
*函数功能:	改变指定电机PWM占空比
*输入参数:
*输出参数:
*备注:
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
				GPIO_ResetBits(GPIOA,GPIO_Pin_5);//将指定位置低 PA4
				GPIO_SetBits(GPIOA,GPIO_Pin_4);	//将指定位置高 PA5

    }
    break;
    case 1: {
				
        TIM_SetCompare2(TIM3,duty);
        TIM_SetCompare1(TIM3,0);
			  GPIO_ResetBits(GPIOA,GPIO_Pin_4);//将指定位置低 PA4
				GPIO_SetBits(GPIOA,GPIO_Pin_5);	//将指定位置高 PA5

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




//电机正转
void Device_Corotation(void)
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);//将指定位置低 PB4
    GPIO_SetBits(GPIOA,GPIO_Pin_5);	//将指定位置高 PB5
}
void Device_corotation(void)
{
    GPIO_ResetBits(GPIOC,GPIO_Pin_5);//将指定位置低
    GPIO_SetBits(GPIOC,GPIO_Pin_4);	//将指定位置高 
}


//电机反转
void Device_Rollback(void)
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_5);//将指定位置低 PB5
    GPIO_SetBits(GPIOA,GPIO_Pin_4);	//将指定位置高 PB4
}
void Device_rollback(void)
{
    GPIO_SetBits(GPIOC,GPIO_Pin_5);
    GPIO_ResetBits(GPIOC,GPIO_Pin_4);
}

//电机停止
void Device_Stop(void)
{
    GPIOA->ODR|=(3<<4);//将指定位置高  PB4 PB5

}
void Device_stop(void)
{
    GPIO_SetBits(GPIOC,GPIO_Pin_5);
    GPIO_SetBits(GPIOC,GPIO_Pin_4);
}


/*********************************************************************
*函数名称:	Change_RotatePWM
*函数功能:	改变指定电机PWM和转向
*输入参数:
*输出参数:
*备注:
*********************************************************************/
void Change_RotatePWM(void)
{

    if(Motor_Pwm>0)
    {
        Change_Motor_PWM(1,absolute_value(Motor_Pwm));//正转
        Device_Corotation();
    }
    else if(Motor_Pwm<0)
    {
        Change_Motor_PWM(2,absolute_value(Motor_Pwm));//反转
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

