#include "sys.h"
#include "delay.h"
//#include "usart.h"
#include "usart2.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "timer.h"
#include "ov7670.h"
#include "ov7725.h"
#include "dcmi.h"
#include "sram.h"
#include "colorcfg.h"
#include "control.h"
#include "sdio_sdcard.h"
//#include "EasyTracered.h"
// ALIENTEK ̽����STM32F407������ ʵ��35
//����ͷ ʵ�� -�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com
//������������ӿƼ����޹�˾
//���ߣ�����ԭ�� @ALIENTEK
//#define  OV7725_WINDOW_WIDTH		240 // <=320		LCD��ʾ��ͼ��ߴ硪���߶�
//#define  OV7725_WINDOW_HEIGHT		320 // <=240		LCD��ʾ��ͼ��ߴ硪�����
extern u8 ov_sta;
u8 sensor = 0;

u8 key = 0, a = 12;
int main(void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����ϵͳ�ж����ȼ�����2
	delay_init(168);								//��ʼ����ʱ����
													//	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	usart2_init(115200);
	LED_Init(); //��ʼ��LED
	LCD_Init(); // LCD��ʼ��
	// 	KEY_Init();					//������ʼ��
	//	TIM3_Int_Init(10000-1,8400-1);//10Khz����,1�����ж�һ��
	FSMC_SRAM_Init();
	POINT_COLOR = RED; //��������Ϊ��ɫ
					   // 	while(SD_Init())//��ⲻ��SD��
					   //	{
					   //		LCD_ShowString(30,150,200,16,16,"SD Card Error!");
					   //		delay_ms(500);
					   //		LCD_ShowString(30,150,200,16,16,"Please Check! ");
					   //		delay_ms(500);
					   //		LED0=!LED0;//DS0��˸
					   //	}

	while (OV7725_Init()) //��ʼ��OV7670
	{
		LCD_ShowString(30, 130, 240, 16, 16, "OV7670 ERR");
		delay_ms(200);
		LCD_Fill(30, 130, 239, 170, WHITE);
		delay_ms(200);
		LED0 = !LED0;
	}

	LCD_ShowString(30, 130, 200, 16, 16, "OV7670 OK");
	delay_ms(1500);

	EXTI8_Init();					//ʹ�ܶ�ʱ������
	OV7725_Window_Set(320, 240, 1); //���ô��� ��ʱ��0���У���

	OV7725_CS = 0;
	LCD_Clear(BLACK);
	while (1)
	{

		//		LCD_DrawRectangle(40,40,160,240);
		OV7725_camera_refresh_color_track(); //������ʾ
		Feature_Recognize_Start_Phoenix();	 /*����ƥ�䲿��*/

		//		static int i=0;
		//		i++;
		//		if(i==15)//DS0��˸.
		//		{
		//			i=0;
		//			LED0=!LED0;
		// 		}
	}
}
