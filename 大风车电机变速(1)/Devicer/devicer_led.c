#include "devicer_led.h"

void LED_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;        //PB6 �������صĿ�������ź�  ָʾ�ɶԻ�е�۽��п���  ������2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;       //PB5  ���͸����صĿɹز��ź� ָʾ���ؿ��Թز� ������3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;       //PB7 ������1
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOB,&GPIO_InitStructure);
		
		

//		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//�ؼ�������JTAG ֻʹ��SWD
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;       //PA15 ���Ƽ��� �������� ������8
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
