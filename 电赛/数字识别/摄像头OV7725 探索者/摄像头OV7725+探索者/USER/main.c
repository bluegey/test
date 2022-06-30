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
// ALIENTEK 探索者STM32F407开发板 实验35
//摄像头 实验 -库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com
//广州市星翼电子科技有限公司
//作者：正点原子 @ALIENTEK
//#define  OV7725_WINDOW_WIDTH		240 // <=320		LCD显示的图像尺寸——高度
//#define  OV7725_WINDOW_HEIGHT		320 // <=240		LCD显示的图像尺寸——宽度
extern u8 ov_sta;
u8 sensor = 0;

u8 key = 0, a = 12;
int main(void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置系统中断优先级分组2
	delay_init(168);								//初始化延时函数
													//	uart_init(115200);		//初始化串口波特率为115200
	usart2_init(115200);
	LED_Init(); //初始化LED
	LCD_Init(); // LCD初始化
	// 	KEY_Init();					//按键初始化
	//	TIM3_Int_Init(10000-1,8400-1);//10Khz计数,1秒钟中断一次
	FSMC_SRAM_Init();
	POINT_COLOR = RED; //设置字体为红色
					   // 	while(SD_Init())//检测不到SD卡
					   //	{
					   //		LCD_ShowString(30,150,200,16,16,"SD Card Error!");
					   //		delay_ms(500);
					   //		LCD_ShowString(30,150,200,16,16,"Please Check! ");
					   //		delay_ms(500);
					   //		LED0=!LED0;//DS0闪烁
					   //	}

	while (OV7725_Init()) //初始化OV7670
	{
		LCD_ShowString(30, 130, 240, 16, 16, "OV7670 ERR");
		delay_ms(200);
		LCD_Fill(30, 130, 239, 170, WHITE);
		delay_ms(200);
		LED0 = !LED0;
	}

	LCD_ShowString(30, 130, 200, 16, 16, "OV7670 OK");
	delay_ms(1500);

	EXTI8_Init();					//使能定时器捕获
	OV7725_Window_Set(320, 240, 1); //设置窗口 有时候0不行？？

	OV7725_CS = 0;
	LCD_Clear(BLACK);
	while (1)
	{

		//		LCD_DrawRectangle(40,40,160,240);
		OV7725_camera_refresh_color_track(); //更新显示
		Feature_Recognize_Start_Phoenix();	 /*特征匹配部分*/

		//		static int i=0;
		//		i++;
		//		if(i==15)//DS0闪烁.
		//		{
		//			i=0;
		//			LED0=!LED0;
		// 		}
	}
}
