#include "devicer_led.h"

void LED_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;        //PB6 接收主控的开舱完毕信号  指示可对机械臂进行控制  传感器2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;       //PB5  发送给主控的可关仓信号 指示主控可以关仓 传感器3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;       //PB7 传感器1
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOB,&GPIO_InitStructure);
		
		

//		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//关键，禁用JTAG 只使用SWD
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;       //PA15 控制夹手 发给主控 传感器8
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//    GPIO_Init(GPIOA,&GPIO_InitStructure);

  
//    GPIO_SetBits(GPIOB,GPIO_Pin_5|GPIO_Pin_7 | GPIO_Pin_6 );

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD,&GPIO_InitStructure);
		  Q_OFF;
		
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;     
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOC,&GPIO_InitStructure);
//		
//		
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;       
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOB,&GPIO_InitStructure);
		
//		GPIO_ResetBits(GPIOD,GPIO_Pin_2 );
//		GPIO_ResetBits(GPIOC,GPIO_Pin_12 );
//		GPIO_ResetBits(GPIOB,GPIO_Pin_5 );

}
