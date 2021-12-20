/*************************************头文件******************************************/
#include "stm32f10x.h"
#include "LCD.h"
#include "UART.h"
#include "LED.h"
#include "SDIO.h"
#include "Touch.h"
#include "EEPROM.h" 
#include "OV7725.h"
#include "string.h"
#include "Control.h"
#include "mydelay.h"

/*************************************宏定义******************************************/
#define Image_Binary_Threshold	145//二值化阈值

#define	SD_INIT_ERROR				1
#define	SD_WRITE_ERROR			2
#define	SD_READ_ERROR				3
#define	SD_ERASE_ERROR			4
#define	CAMERA_INIT_ERROR		5

/************************************全局变量******************************************/
Storage_Info_TypeDef storage_info;
Image_Info_TypeDef image_info;

SD_Error Status = SD_ERROR;
uint8_t try_count=5;
uint8_t camera_buff_select_temp;
uint16_t block_write_count=0;
uint8_t menuitem=0;
uint16_t LED_luminance;


extern uint8_t camera_buff_select;
extern uint8_t camera_buff_empty[2];
extern uint8_t camera_buff[2][10240];	

uint8_t image_buff[64][550]={0};
uint8_t image_border[360]={0};
//uint8_t image_error_flag=0;
uint16_t start_location;
uint8_t key_tepe;

/************************************函数声明******************************************/
void UART_Send_Image(void);
void Error_Handler(uint8_t error_code);
uint8_t Display_main_view(void);
uint8_t Display_system_set(void);
uint8_t Display_new_key(void);
uint8_t Display_old_key(void);
void Display_about_project(void);
void Display_brightness_set(void);
void Display_help(void);
void Incise_Key(void);
void LCD_Show_Image(uint32_t address);
void Image_Collect(uint32_t address);
uint8_t Image_Process(uint32_t address);
uint16_t Allot_User_serialnum(void);
void Operate_User_serialnum(uint8_t opt,uint16_t serialnum);
void Image_Info_Read(uint32_t address,uint8_t* pbuff,uint8_t len);
void Image_Info_Save(uint32_t address,uint8_t* pbuff,uint8_t len);
uint32_t Storage_Section_Check(uint32_t address);

/**************************************************************************************/

//FSMC_NORSRAMInit
u8 d_falgw;
int main()
{

	Control_Init();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	UART_Init();
	AT24CXX_Init();
	LCD_Init();	
//	LCD_string_font=ASCII_0816;
//	LCD_Display_Message(300,300 ,(u8*)"A");
	tp_dev.init();     
//  LED_Init();
 
	while(try_count--)
	{
		if(SD_OK == (Status = SD_Init()))
			break;
	}
	if(Status != SD_OK)
	{
		Error_Handler(SD_INIT_ERROR);
	}
//	
	if(OV7725_init())
	{
		Error_Handler(CAMERA_INIT_ERROR);
	}
//	

	Platform_initial_position();
	AT24CXX_Read(16,(uint8_t*)(&LED_luminance),2);
	TIM_SetCompare4(TIM3,994);
	SD_Erase(0,512*10);
	storage_info.total=0;
	storage_info.first=0;
	storage_info.last=0; 
	AT24CXX_Write(20,(uint8_t*)(&storage_info),10);
	
	while(1)
	{
		menuitem=Display_main_view();
		if(menuitem==1)//配新钥匙
		{
			while(1)
			{
				menuitem=Display_new_key();
				if(menuitem==0)
					break;
			}
		}
		else if(menuitem==2)//配旧钥匙
		{
			while(1)
			{
				menuitem=Display_old_key();
				if(menuitem==0)
					break;
			}
		}
		else if(menuitem==3)//系统设置
		{
			while(1)
			{
				menuitem=Display_system_set();
				if(menuitem==1)//灯光调节
				{
					Display_brightness_set();
				}
				else if(menuitem==2)//触屏校准
				{
					tp_dev.adjust();
				}
				else if(menuitem==3)//关于项目
				{
					Display_about_project();
				}
				else if(menuitem==0)//返回
				{
					break;
				}
			}
		}
		else if(menuitem==4)//帮助
		{
			Display_help();
		}		
	}

}
void Error_Handler(uint8_t error_code)
{
	if(error_code==SD_INIT_ERROR)
	{
		LCD_Clear(WHITE);
		LCD_board_color=WHITE;
		LCD_chinese_font=Chinese_3232;
		LCD_string_font=ASCII_1632;
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(180,120,644,357);
		LCD_DrawRectangle(181,121,643,356);
		
		LCD_brush_color=RED;
//		LCD_Display_String(300,150,"123156");
		LCD_Display_Message(204,200,"请重新启动或者关机检查SD卡");
		
		LCD_Display_Message(204,300,"重启");
		LCD_Display_Message(556,300,"关机");
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(199,295,273,337);
		LCD_DrawRectangle(200,296,272,336);//重启边框
		LCD_DrawRectangle(551,295,625,337);
		LCD_DrawRectangle(552,296,624,336);//关机边框
		
		TP_IRQ_ENABLE();//开启触控
		while(1)
		{
			if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
			{		
				tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
				
				if(TP_Area_Press_Judge(200,295,270,335))//重启按键被按下
				{
					LCD_Clear(BLACK);
					NVIC_SystemReset();
				}
				else if(TP_Area_Press_Judge(555,295,625,335))//关机按键被按下
				{
					LCD_Clear(BLACK);
					SYSTEM_OFF();
					while(1);
				}
			}
		}
	}
	else if(error_code==CAMERA_INIT_ERROR)
	{
		LCD_Clear(WHITE);
		LCD_board_color=WHITE;
		LCD_chinese_font=Chinese_3232;
		LCD_string_font=ASCII_1632;
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(180,120,676,357);
		LCD_DrawRectangle(181,121,675,356);
		
		LCD_brush_color=RED;
		LCD_Display_Message(300,150,"摄像头初始化失败！");
		LCD_Display_Message(204,200,"请重新启动或者关机检查摄像头");
		
		LCD_Display_Message(204,300,"重启");
		LCD_Display_Message(588,300,"关机");
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(199,295,273,337);
		LCD_DrawRectangle(200,296,272,336);//重启边框
		LCD_DrawRectangle(582,295,657,337);
		LCD_DrawRectangle(583,296,656,336);//关机边框
		
		TP_IRQ_ENABLE();//开启触控
		while(1)
		{
			if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
			{		
				tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
				
				if(TP_Area_Press_Judge(200,295,270,335))//重启按键被按下
				{
					LCD_Clear(BLACK);
					NVIC_SystemReset();
				}
				else if(TP_Area_Press_Judge(585,295,655,335))//关机按键被按下
				{
					LCD_Clear(BLACK);
					SYSTEM_OFF();
					while(1);
				}
			}
		}
	}
	else if(error_code==SD_WRITE_ERROR)
	{
		LCD_Clear(WHITE);
		LCD_board_color=WHITE;
		LCD_chinese_font=Chinese_3232;
		LCD_string_font=ASCII_1632;
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(180,120,644,357);
		LCD_DrawRectangle(181,121,643,356);
		
		LCD_brush_color=RED;
		LCD_Display_Message(316,150,"SD卡写入失败！");
		LCD_Display_Message(204,200,"请重新启动或者关机检查SD卡");
		
		LCD_Display_Message(204,300," ");
		LCD_Display_Message(556,300,"关机");
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(199,295,273,337);
		LCD_DrawRectangle(200,296,272,336);//重启边框
		LCD_DrawRectangle(551,295,625,337);
		LCD_DrawRectangle(552,296,624,336);//关机边框
		
		TP_IRQ_ENABLE();//开启触控
		while(1)
		{
			if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
			{		
				tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
				
				if(TP_Area_Press_Judge(200,295,270,335))//重启按键被按下
				{
					LCD_Clear(BLACK);
					NVIC_SystemReset();
				}
				else if(TP_Area_Press_Judge(555,295,625,335))//关机按键被按下
				{
					LCD_Clear(BLACK);
					SYSTEM_OFF();
					while(1);
				}
			}
		}
	}
	else if(error_code==SD_READ_ERROR)
	{
		LCD_Clear(WHITE);
		LCD_board_color=WHITE;
		LCD_chinese_font=Chinese_3232;
		LCD_string_font=ASCII_1632;
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(180,120,644,357);
		LCD_DrawRectangle(181,121,643,356);
		
		LCD_brush_color=RED;
		LCD_Display_Message(316,150,"SD卡读取失败！");
		LCD_Display_Message(204,200,"请重新启动或者关机检查SD卡");
		
		LCD_Display_Message(204,300,"重启");
		LCD_Display_Message(556,300,"关机");
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(199,295,273,337);
		LCD_DrawRectangle(200,296,272,336);//重启边框
		LCD_DrawRectangle(551,295,625,337);
		LCD_DrawRectangle(552,296,624,336);//关机边框
		
		TP_IRQ_ENABLE();//开启触控
		while(1)
		{
			if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
			{		
				tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
				
				if(TP_Area_Press_Judge(200,295,270,335))//重启按键被按下
				{
					LCD_Clear(BLACK);
					NVIC_SystemReset();
				}
				else if(TP_Area_Press_Judge(555,295,625,335))//关机按键被按下
				{
					LCD_Clear(BLACK);
					SYSTEM_OFF();
					while(1);
				}
			}
		}
	}
}


uint8_t Display_main_view(void)
{
	uint8_t temp;
	LCD_Clear(WHITE);
	LCD_Fill(0,0,854,92,BLUE);
	LCD_board_color=BLUE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(171,30,"全 自 动 数 字 钥 匙 配 取 装 置");
	
	LCD_Fill(333,104,521,196,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,134,"配新钥匙");
	
	LCD_Fill(333,204,521,296,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,234,"配旧钥匙");
	
	LCD_Fill(333,304,521,396,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,334,"系统设置");
	
	LCD_Fill(0,428,84,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(10,438,"帮助");
	
	LCD_Fill(770,428,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(780,438,"关机");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_string_font=ASCII_1224;
	LCD_Display_Message(193,446,"电子信息工程学院---工程训练中心 2015.12");
	
	TP_IRQ_ENABLE();//开启触控
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
		{		
			tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
			
			if(TP_Area_Press_Judge(333,104,521,196))//配新钥匙按键被按下
			{
				temp=1;
				break;
			}
			else if(TP_Area_Press_Judge(333,204,521,296))//配旧钥匙按键被按下
			{
				temp=2;
				break;
			}
			else if(TP_Area_Press_Judge(333,304,521,396))//系统设置按键被按下
			{
				temp=3;
				break;
			}
			else if(TP_Area_Press_Judge(0,436,68,480))//帮助按键被按下
			{
				temp=4;
				break;
			}
			else if(TP_Area_Press_Judge(786,436,854,480))//关机按键被按下
			{
				LCD_Clear(BLACK);
				SYSTEM_OFF();
				while(1);
			}
		}
	}
	TP_IRQ_DISABLE();
	return temp;
}

uint8_t Display_system_set(void)
{
	uint8_t temp;
	LCD_Clear(WHITE);
	LCD_Fill(0,0,854,92,BLUE);
	LCD_board_color=BLUE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(50,30,"系  统  设  置");
	
	LCD_Fill(333,104,521,196,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,134,"灯光调节");
	
	LCD_Fill(333,204,521,296,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,234,"触屏校准");
	
	LCD_Fill(333,304,521,396,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,334,"关于项目");
	
	LCD_Fill(786,436,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(796,446,"返回");
	
	TP_IRQ_ENABLE();//开启触控
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
		{		
			tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
			
			if(TP_Area_Press_Judge(333,104,521,196))//灯光调节按键被按下
			{
				temp=1;
				break;
			}
			else if(TP_Area_Press_Judge(333,204,521,296))//触屏校准按键被按下
			{
				temp=2;
				break;
			}
			else if(TP_Area_Press_Judge(333,304,521,396))//关于项目按键被按下
			{
				temp=3;
				break;
			}
			else if(TP_Area_Press_Judge(786,436,854,480))//返回按键被按下
			{
				temp=0;
				break;
			}
		}
	}
	TP_IRQ_DISABLE();
	return temp;
}


uint8_t Display_new_key(void)
{
	uint8_t collect_success_flag;
	uint32_t add;
	uint16_t num_1;
	uint16_t count_1;
	
	LCD_Clear(WHITE);
	LCD_Fill(0,0,854,92,BLUE);
	LCD_board_color=BLUE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(50,30,"配 新 钥 匙");
	
	LCD_board_color=WHITE;
	LCD_brush_color=MAGENTA;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,120,"步骤 1 :");
	LCD_Display_Chinese(30,160,"图像采集处理");
	LCD_Display_Message(400,140,"正在采集图像...");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,255,"步骤 2 :");
	LCD_Display_Chinese(30,295,"开始配取钥匙");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,390,"步骤 3 :");
	LCD_Display_Chinese(30,430,"保存钥匙数据");
	
	
	//第一步图像采集处理，图像处理失败后询问是否需要重新采集（默认）。图像处理成功后开始进行切割（默认）
	AT24CXX_Read(20,(uint8_t*)(&storage_info),10);//读取最后一张图片地址
	add=Storage_Section_Check(storage_info.last+409600);//
	collect_success_flag=0;
	while(!collect_success_flag)
	{
		Image_Collect(add);//采集图像
		if(Image_Process(add))//图像处理失败，询问是否需要重新采集
		{
			count_1=200;
			
			LCD_board_color=WHITE;
			LCD_brush_color=RED;
			LCD_chinese_font=Chinese_3232;
			LCD_string_font=ASCII_1632;
			LCD_Display_Chinese(397,250,	   "图像采集处理失败");
			LCD_Display_Chinese(317,290,"请检查钥匙后重新采集或返回");
			
			LCD_Display_Chinese(678,410,"返回");
			LCD_Display_Message(308,410,"重试(9s)");
			
			LCD_DrawRectangle(295,230,755,460);
			LCD_DrawRectangle(303,400,441,452);
			LCD_DrawRectangle(673,400,747,452);
			
			TP_IRQ_ENABLE();
			while(1)
			{
				if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
				{		
					tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
					
					if(TP_Area_Press_Judge(303,400,441,452))//重试按键被按下
					{
						TP_IRQ_DISABLE();
						LCD_Fill(240,120,810,470,WHITE);
						break;
					}
					else if(TP_Area_Press_Judge(673,400,747,452))//返回按键被按下
					{
						TP_IRQ_DISABLE();
						return 0;
					}
				}
				count_1--;
				LCD_Display_Format(388,410,"%1d",count_1/20);
				my_delayms(50);
				if(count_1<5)
				{
					TP_IRQ_DISABLE();
					LCD_Fill(240,120,810,470,WHITE);
					break;
				}
			}
		}
		else
		{
			count_1=120;
			LCD_brush_color=GREEN;
			LCD_DrawRectangle(249,129,801,195);
			
			LCD_board_color=WHITE;
			LCD_brush_color=RED;
			LCD_chinese_font=Chinese_3232;
			LCD_string_font=ASCII_1632;
			LCD_Display_Chinese(397,250,	  "图像采集处理成功");
			LCD_Display_Chinese(323,290,"请确认是否使用图像或返回");
			
			LCD_Display_Chinese(678,410,"返回");
			LCD_Display_Message(308,410,"确定(5s)");
			
			LCD_DrawRectangle(295,230,755,460);
			LCD_DrawRectangle(303,400,441,452);
			LCD_DrawRectangle(673,400,747,452);
			
			TP_IRQ_ENABLE();
			while(1)
			{
				if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
				{		
					tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
					
					if(TP_Area_Press_Judge(303,400,441,452))//确定按键被按下
					{
						TP_IRQ_DISABLE();
//						LCD_Fill(240,120,810,470,WHITE);
						collect_success_flag=1;
						break;
					}
					else if(TP_Area_Press_Judge(673,400,747,452))//返回按键被按下
					{
						TP_IRQ_DISABLE();
						return 0;
					}
				}
				count_1--;
				LCD_Display_Format(388,410,"%1d",count_1/20);
				my_delayms(50);
				if(count_1<5)
				{
					TP_IRQ_DISABLE();
//					LCD_Fill(240,120,810,470,WHITE);
					collect_success_flag=1;
					break;
				}
			}
		}
	}
	
	//第二步先检查平台是否在安全位置，然后将平台移动到工作位置，根据情况查看是否需要微调（默认不需要），最后开始进行切割，切割完成后恢复平台到安全位置
	LCD_board_color=WHITE;
	LCD_brush_color=LGRAY;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,120,"步骤 1 :");
	LCD_Display_Chinese(30,160,"图像采集处理");
	
	LCD_board_color=WHITE;
	LCD_brush_color=MAGENTA;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,255,"步骤 2 :");
	LCD_Display_Chinese(30,295,"开始配取钥匙");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,390,"步骤 3 :");
	LCD_Display_Chinese(30,430,"保存钥匙数据");
	
	LCD_Fill(290,225,760,465,WHITE);
	LCD_brush_color=RED;
	LCD_board_color=WHITE;
	LCD_DrawRectangle(295,230,755,460);
	LCD_Display_Message(333,329,"正在切割钥匙,请注意安全!");
	
	Incise_Key();
	
	//第三步询问是否保存用户数据，默认不保存
	LCD_board_color=WHITE;
	LCD_brush_color=LGRAY;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,120,"步骤 1 :");
	LCD_Display_Chinese(30,160,"图像采集处理");
	
	LCD_board_color=WHITE;
	LCD_brush_color=LGRAY;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,255,"步骤 2 :");
	LCD_Display_Chinese(30,295,"开始配取钥匙");
	
	LCD_board_color=WHITE;
	LCD_brush_color=MAGENTA;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,390,"步骤 3 :");
	LCD_Display_Chinese(30,430,"保存钥匙数据");	
	
	LCD_Fill(240,100,840,470,WHITE);
	
	LCD_board_color=WHITE;
	LCD_brush_color=RED;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(301,150,"请取出钥匙, 是否保存用户数据");
	LCD_Display_Message(413,200,"用户编号: ");
	num_1=Allot_User_serialnum();
	LCD_Display_Format(587,200,"%4d",num_1);
	
	LCD_Display_Message(285,393,"保存");
	LCD_Display_Message(637,393,"取消(9s)");
	
	LCD_DrawRectangle(270,130,780,440);
	LCD_DrawRectangle(280,388,354,430);
	LCD_DrawRectangle(632,388,770,430);
	
	TP_IRQ_ENABLE();
	count_1=200;
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
		{		
			tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
			
			if(TP_Area_Press_Judge(280,388,354,430))//保存按键被按下
			{
				
				
				if(storage_info.total==0)
				{
					if(key_tepe==1)
						image_info.type='B';
					else if(key_tepe==2)
						image_info.type='S';
					
					image_info.serial_num=1;
					image_info.prior=0;
					image_info.next=0;
					image_info.self=add;
					
					storage_info.first=add;
					storage_info.last=add;
					storage_info.total=1;
					
					Image_Info_Save(add,(uint8_t*)(&image_info),16);
					AT24CXX_Write(20,(uint8_t*)(&storage_info),10);
					Operate_User_serialnum(0,1);
				}
				else
				{
					Image_Info_Read(storage_info.last,(uint8_t*)(&image_info),16);
					image_info.next=add;
					Image_Info_Save(storage_info.last,(uint8_t*)(&image_info),16);
					
					storage_info.total++;
					
					if(key_tepe==1)
						image_info.type='B';
					else if(key_tepe==2)
						image_info.type='S';
					
					image_info.serial_num=num_1;
					image_info.prior=storage_info.last;
					image_info.next=0;
					image_info.self=add;
					
					storage_info.last=add;
					
					Image_Info_Save(add,(uint8_t*)(&image_info),16);
					AT24CXX_Write(20,(uint8_t*)(&storage_info),10);
					Operate_User_serialnum(0,num_1);
				}
				break;
			}
			else if(TP_Area_Press_Judge(632,388,770,430))//取消按键被按下
			{
				break;
			}
		}
		count_1--;
		LCD_Display_Format(717,393,"%1d",count_1/20);
		my_delayms(50);
		if(count_1<5)
		{
			break;
		}		
	}
	TP_IRQ_DISABLE();
	
	return 0;
}


uint8_t Display_old_key(void)
{
	uint16_t page=1,select_num=1;
	Image_Info_TypeDef image_info_temp[9];
	
	LCD_Clear(WHITE);
	LCD_Fill(0,0,854,92,BLUE);
	LCD_board_color=BLUE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(50,30,"配 旧 钥 匙");
	
	LCD_Fill(786,436,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(796,446,"返回");
	
	AT24CXX_Read(20,(uint8_t*)(&storage_info),10);

	if(storage_info.total==0)//无用户数据
	{
		LCD_board_color=WHITE;
		LCD_brush_color=RED;
		LCD_string_font=ASCII_1632;
		LCD_chinese_font=Chinese_3232;
		LCD_Display_Chinese(331,230,"没有用户数据");
		LCD_DrawRectangle(250,180,604,312);
		
		TP_IRQ_ENABLE();
		while(1)
		{
			if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
			{		
				tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
				
				if(TP_Area_Press_Judge(786,436,854,480))//返回按键被按下
				{
					TP_IRQ_DISABLE();
					return 0;
				}
			}
		}
	}
	else
	{
		uint8_t i;
		
		LCD_board_color=WHITE;
		LCD_brush_color=RED;
		LCD_chinese_font=Chinese_2424;
		LCD_Display_Chinese(64,114,"上一页");
		LCD_Display_Chinese(64,434,"下一页");
		LCD_DrawRectangle(20,107,180,145);
		LCD_DrawRectangle(20,427,180,465);
		
		LCD_board_color=WHITE;
		LCD_brush_color=DGREEN;
		LCD_chinese_font=Chinese_3232;
		LCD_string_font=ASCII_1632;
		LCD_Display_Chinese(429,250,"使用模板配取");
		LCD_Display_Chinese(429,360,"删除用户数据");
		LCD_DrawRectangle(350,240,700,292);
		LCD_DrawRectangle(350,350,700,402);
		
		LCD_board_color=CYAN;
		LCD_brush_color=BLACK;
		LCD_chinese_font=Chinese_3232;
		LCD_string_font=ASCII_1632;
		LCD_Display_Message(20,150,"编号:");
		Image_Info_Read(storage_info.first,(uint8_t*)(&image_info_temp[0]),16);
		LCD_Display_Format(100,150,"%c%d",image_info_temp[0].type,image_info_temp[0].serial_num);
		Image_Process(image_info_temp[0].self);
		
		
		LCD_board_color=WHITE;
		LCD_brush_color=BLACK;
		LCD_chinese_font=Chinese_3232;
		LCD_string_font=ASCII_1632;
		
		for(i=1;i<storage_info.total;i++)
		{
			Image_Info_Read(image_info_temp[i-1].next,(uint8_t*)(&image_info_temp[i]),16);
			LCD_Display_Message(20,150+40*i,"编号:");
			LCD_Display_Format(100,150+40*i,"%c%d",image_info_temp[i].type,image_info_temp[i].serial_num);
			if(i==6)
				break;
		}
		
	}
	my_delayms(1000);
	TP_IRQ_ENABLE();
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
		{		
			TP_IRQ_DISABLE();
			tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
			if(TP_Area_Press_Judge(786,436,854,480))//返回按键被按下
			{
				return 0;
			}
			else if(TP_Area_Press_Judge(350,240,700,292))//配取按键被按下
			{
				LCD_Fill(290,225,760,465,WHITE);
				LCD_chinese_font=Chinese_3232;
				LCD_string_font=ASCII_1632;
				LCD_brush_color=RED;
				LCD_board_color=WHITE;
				LCD_DrawRectangle(295,230,755,460);
				LCD_Display_Message(333,329,"正在切割钥匙,请注意安全!");
	
				Incise_Key();
				return 1;
			}
			else if(TP_Area_Press_Judge(350,350,700,402))//删除按键被按下
			{
				if((page-1)*7+select_num==1)//删除第一个数据
				{
					if(storage_info.total==1)
					{
						SD_Erase(0,512*10);
						storage_info.total=0;
						storage_info.first=0;
						storage_info.last=0;
						AT24CXX_Write(20,(uint8_t*)(&storage_info),10);
					}
					else
					{
						storage_info.total--;
						storage_info.first=image_info_temp[1].self;
						
						image_info_temp[1].prior=0;
						
						Operate_User_serialnum(1,image_info_temp[0].serial_num);
						Image_Info_Save(image_info_temp[1].self,(uint8_t*)(&image_info_temp[1]),16);
						AT24CXX_Write(20,(uint8_t*)(&storage_info),10);
					}
				}
				else if((page-1)*7+select_num==storage_info.total)//删除最后一个数据
				{
					Image_Info_Read(image_info_temp[select_num-1].prior,(uint8_t*)(&image_info_temp[7]),16);
					
					storage_info.total--;
					storage_info.last=image_info_temp[7].self;
					
					image_info_temp[7].next=0;
					
					Operate_User_serialnum(1,image_info_temp[select_num-1].serial_num);
					Image_Info_Save(image_info_temp[7].self,(uint8_t*)(&image_info_temp[7]),16);
					AT24CXX_Write(20,(uint8_t*)(&storage_info),10);
				}
				else
				{
					Image_Info_Read(image_info_temp[select_num-1].prior,(uint8_t*)(&image_info_temp[7]),16);
					Image_Info_Read(image_info_temp[select_num-1].next,(uint8_t*)(&image_info_temp[8]),16);
					
					storage_info.total--;
					
					image_info_temp[7].next=image_info_temp[8].self;
					image_info_temp[8].prior=image_info_temp[7].self;
					
					Operate_User_serialnum(1,image_info_temp[select_num-1].serial_num);
					Image_Info_Save(image_info_temp[7].self,(uint8_t*)(&image_info_temp[7]),16);
					Image_Info_Save(image_info_temp[8].self,(uint8_t*)(&image_info_temp[8]),16);
					AT24CXX_Write(20,(uint8_t*)(&storage_info),10);
				}
				return 1;
			}
			else if(TP_Area_Press_Judge(20,107,180,145))//上一页按键被按下
			{
				if(page>1)
				{
					uint8_t ii;
					page--;
					select_num=1;
					
					Image_Info_Read(image_info_temp[0].prior,(uint8_t*)(&image_info_temp[6]),16);
					for(ii=6;ii>0;ii--)
						Image_Info_Read(image_info_temp[ii].prior,(uint8_t*)(&image_info_temp[ii-1]),16);
					
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,150,"编号:");
					LCD_Display_Format(100,150,"%c%d",image_info_temp[0].type,image_info_temp[0].serial_num);
					Image_Process(image_info_temp[0].self);
					
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					
					for(ii=1;ii<7;ii++)
					{
						LCD_Display_Message(20,150+40*ii,"编号:");
						LCD_Display_Format(100,150+40*ii,"%c%d",image_info_temp[ii].type,image_info_temp[ii].serial_num);
					}
				}
			}
			else if(TP_Area_Press_Judge(20,427,180,465))//下一页按键被按下
			{
				if(page<((storage_info.total-1)/7+1))
				{
					uint16_t ii;
					uint8_t jj;
					page++;
					select_num=1;
					LCD_Fill(19,149,181,423,WHITE);
					
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,150,"编号:");
					Image_Info_Read(image_info_temp[6].next,(uint8_t*)(&image_info_temp[0]),16);
					LCD_Display_Format(100,150,"%c%d",image_info_temp[0].type,image_info_temp[0].serial_num);
					Image_Process(image_info_temp[0].self);
					
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					
					for(ii=1+7*page,jj=1;ii<storage_info.total;ii++,jj++)
					{
						Image_Info_Read(image_info_temp[jj-1].next,(uint8_t*)(&image_info_temp[jj]),16);
						LCD_Display_Message(20,150+40*jj,"编号:");
						LCD_Display_Format(100,150+40*jj,"%c%d",image_info_temp[jj].type,image_info_temp[jj].serial_num);
						if(jj==6)
							break;
					}
				}
			}
			else if(TP_Area_Press_Judge(20,146,180,186))//1按键被按下
			{
				if((((page-1)*7+1)<=storage_info.total)&&(select_num!=1))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=1;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,186,180,226))//2按键被按下
			{
				if((((page-1)*7+2)<=storage_info.total)&&(select_num!=2))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=2;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,226,180,266))//3按键被按下
			{
				if((((page-1)*7+3)<=storage_info.total)&&(select_num!=3))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=3;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,266,180,306))//4按键被按下
			{
				if((((page-1)*7+4)<=storage_info.total)&&(select_num!=4))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=4;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,306,180,346))//5按键被按下
			{
				if((((page-1)*7+5)<=storage_info.total)&&(select_num!=5))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=5;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,346,180,386))//6按键被按下
			{
				if((((page-1)*7+6)<=storage_info.total)&&(select_num!=6))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=6;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,386,180,426))//7按键被按下
			{
				if((((page-1)*7+7)<=storage_info.total)&&(select_num!=7))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=7;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"编号:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			TP_IRQ_ENABLE();
		}
	}
	
	return 0;
}

void Display_about_project(void)
{
	uint8_t value=0;
	LCD_Clear(GREEN);
	LCD_Fill(0,0,854,92,BLUE);
	LCD_board_color=BLUE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(50,30,"关 于 项 目");
	
	LCD_board_color=GREEN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	
	LCD_Display_Message(50,120,"项目名称:全自动数字钥匙配取装置");
	LCD_Display_Message(50,180,"项目类别:国家级大学生创新创业训练计划项目");
	LCD_Display_Message(50,240,"项目成员:张涛,鄢川杰,赵阳,张明峰,刘俊");
	LCD_Display_Message(50,300,"指导教师:苗志全");
	
	LCD_chinese_font=Chinese_2424;
	LCD_string_font=ASCII_1224;
	LCD_Display_Message(211,380,"***电子信息工程学院--工程训练中心***");
	
	LCD_Fill(786,436,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(796,446,"返回");
	
//	LCD_DrawRectangle(377,50,477,150);
//	LCD_DrawRectangle(377,330,477,430);
//	LCD_DrawRectangle(227,190,327,290);
//	LCD_DrawRectangle(527,190,627,290);
	
	TP_IRQ_ENABLE();//开启触控
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
		{	
			TP_IRQ_DISABLE();
			tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
			if(TP_Area_Press_Judge(786,436,854,480))//返回按键被按下
			{
				break;
			}
			else if(TP_Area_Press_Judge(377,50,477,150))//返回按键被按下
			{
				value=1;
			}
			else if(TP_Area_Press_Judge(527,190,627,290))//返回按键被按下
			{
				if(value==1)
					value=2;
			}
			else if(TP_Area_Press_Judge(377,330,477,430))//返回按键被按下
			{
				if(value==2)
					value=3;
			}
			else if(TP_Area_Press_Judge(227,190,327,290))//返回按键被按下
			{
				if(value==3)
				{
					LCD_Clear(BLUE);
					my_delayms(5000);
					break;
				}
			}
			TP_IRQ_ENABLE();
		}
	}
	TP_IRQ_DISABLE();
}


void Display_brightness_set(void)
{
	LCD_Clear(WHITE);
	LCD_Fill(0,0,854,92,BLUE);
	LCD_board_color=BLUE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(50,30,"灯 光 调 节");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(760,100,"亮");
	LCD_Display_Chinese(760,396,"暗");
	
	LCD_Fill(786,436,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(796,446,"返回");

	LCD_Fill(720,100,750,420,LGRAY);
	LCD_Fill(730,110,740,410,0x4208);
	LCD_Fill(725,255,745,265,DGRAY);
	
	LCD_brush_color=GREEN;
	LCD_DrawRectangle(19,99,661,421);
	
	
	Image_Collect((uint32_t)409600*9000);
	LCD_Show_Image((uint32_t)409600*9000);
	TP_IRQ_ENABLE();//开启触控
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
		{		
			TP_IRQ_DISABLE();
			tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
			if(TP_Area_Press_Judge(786,436,854,480))//返回按键被按下
			{
				break;
			}
			else if(TP_Area_Press_Judge(710,110,760,410))//灯光亮暗调节
			{
				LCD_Fill(720,100,750,420,LGRAY);
				LCD_Fill(730,110,740,410,0x4208);		
				LCD_Fill(725,tp_dev.y,745,tp_dev.y+10,DGRAY);	

				LED_luminance=(tp_dev.y-110)/30+989;
				TIM_SetCompare4(TIM3,LED_luminance);
				AT24CXX_Write(16,(uint8_t*)(&LED_luminance),2);
				
				Image_Collect((uint32_t)409600*9000);
				LCD_Show_Image((uint32_t)409600*9000);
			}
			TP_IRQ_ENABLE();
		}
	}
	TP_IRQ_DISABLE();
}


void Display_help(void)
{
	LCD_Clear(WHITE);
	LCD_Fill(0,0,854,92,BLUE);
	LCD_board_color=BLUE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(50,30,"帮  助");
	
	LCD_Fill(786,436,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(796,446,"返回");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,150,"配新钥匙:用户首次使用本设备配取钥匙");
	LCD_Display_Message(20,250,"配旧钥匙:用户使用保存的数据配取钥匙");
	LCD_Display_Message(20,350,"系统设置:设置数据采集时灯光强度,触屏校准等参数");
	
	TP_IRQ_ENABLE();//开启触控
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
		{		
			tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
			
			if(TP_Area_Press_Judge(786,436,854,480))//返回按键被按下
			{
				break;
			}
		}
	}
	TP_IRQ_DISABLE();
}

void LCD_Show_Image(uint32_t address)
{
	uint16_t i,j,k;
	
	LCD_Set_Window(20,100,660,420);
	LCD_WriteRAM_Prepare();
	
	for(j=40;j<440;j+=20)
	{
		k=20;
		while(--k)
		{
			if(SD_OK==SD_ReadMultiBlocks(camera_buff[0],address+512*j,512,20))
				break;
			else
			{
				SD_Init();
				my_delayms(50);
			}
		}
		if(k==0)
			Error_Handler(SD_READ_ERROR);
		
		my_delayms(2);
		for(i=0;i<10240;i++)
		{
			if(camera_buff[0][i]>Image_Binary_Threshold)
				LCD_Write_Data(WHITE);
			else
				LCD_Write_Data(BLACK);
		}
	}
}



void Image_Collect(uint32_t address)
{
	block_write_count=0;
	
	while(SD_OK!=SD_Erase(address,address+409600));
//开启行中断	
	EXTI_ClearITPendingBit(EXTI_Line0);
	NVIC_EnableIRQ(EXTI0_IRQn);
	
	while(1)
	{
		camera_buff_select_temp=camera_buff_select^0x01;//
		if(camera_buff_empty[camera_buff_select_temp]==0)
		{
			if(SD_OK!=SD_WriteMultiBlocks(camera_buff[camera_buff_select_temp],address+512*block_write_count,512,20))
				Error_Handler(SD_WRITE_ERROR);
			camera_buff_empty[camera_buff_select_temp]=1;
			block_write_count+=20;
			if(block_write_count==600)
			{
				break;
			}
		}
	}
}

uint8_t Image_Process(uint32_t address)
{
	uint16_t i,j;
	uint8_t k,x;
	int8_t zero_one_num;
	uint16_t count=0;
	uint16_t length;
	
	//从SD卡中读取可用数据
	//判断是不是大钥匙
	key_tepe=1;
	length=350;
	x=0;
	for(j=230;j<310;j+=10)
	{
		k=20;
		while(--k)
		{
			if(SD_OK==SD_ReadMultiBlocks(camera_buff[0],address+512*j,512,10))
				break;
			else
			{
				SD_Init();
				my_delayms(50);
			}
		}
		if(k==0)
			Error_Handler(SD_READ_ERROR);
		
		my_delayms(5);
		for(i=0;i<5120;i++)
		{
			if(i%640<550)
				image_buff[x+i/640][i%640]=camera_buff[0][i];
		}
		x+=8;//
	} 
	
	//寻找钥匙起始位置
	//钥匙起始位置一般在第40-63行，第420-520列
	start_location=0;
	for(i=63;i>58;i--)
	{
		for(j=410;j<460;j++)
		{
			zero_one_num=0;
			if(image_buff[i][j-1]>Image_Binary_Threshold)
				zero_one_num++;
			else
				zero_one_num--;
			
			if(image_buff[i][j]>Image_Binary_Threshold)
				zero_one_num++;
			else
				zero_one_num--;
			
			if(image_buff[i][j+1]>Image_Binary_Threshold)
				zero_one_num++;
			else
				zero_one_num--;
			
			if(image_buff[i][j+2]>Image_Binary_Threshold)
				zero_one_num++;
			else
				zero_one_num--;
			
			if(zero_one_num==0)
			{
				start_location+=j;
				break;
			}
		}
	}

	start_location/=5;
	if(start_location<410||start_location>460)
	{
		key_tepe=2;//不是大钥匙，继续判断是不是小钥匙
		length=265;
	}
	
	//判断是不是小钥匙
	if(key_tepe==2)
	{
		x=0;
		for(j=200;j<280;j+=10)
		{
			k=20;
			while(--k)
			{
				if(SD_OK==SD_ReadMultiBlocks(camera_buff[0],address+512*j,512,10))
					break;
				else
				{
					SD_Init();
					my_delayms(50);
				}
			}
			if(k==0)
				Error_Handler(SD_READ_ERROR);
			
			my_delayms(5);
			for(i=0;i<5120;i++)
			{
				if(i%640<550)
					image_buff[x+i/640][i%640]=camera_buff[0][i];
			}
			x+=8;
		}
		
		//寻找钥匙起始位置
		//钥匙起始位置一般在第40-63行，第420-520列
		start_location=0;
		for(i=63;i>58;i--)
		{
			for(j=410;j<460;j++)
			{
				zero_one_num=0;
				if(image_buff[i][j-1]>Image_Binary_Threshold)
					zero_one_num++;
				else
					zero_one_num--;
				
				if(image_buff[i][j]>Image_Binary_Threshold)
					zero_one_num++;
				else
					zero_one_num--;
				
				if(image_buff[i][j+1]>Image_Binary_Threshold)
					zero_one_num++;
				else
					zero_one_num--;
				
				if(image_buff[i][j+2]>Image_Binary_Threshold)
					zero_one_num++;
				else
					zero_one_num--;
				
				if(zero_one_num==0)
				{
					start_location+=j;
					break;
				}
			}
		}

		start_location/=5;
		if(start_location<410||start_location>460)
		{
			key_tepe=0;
			return 1;
		}
	}
	
	//从起始位置开始，寻找齿边缘
	//钥匙总长度大约350个像素点
	for(j=start_location-1;j>start_location-length-1;j--)
	{
		if(j<start_location-start_location-5)
		{
			image_border[count]=image_border[count-1];
		}
		else
			image_border[count]=1;
		for(i=60;i>1;i--)
		{
			zero_one_num=0;
			
			if(image_buff[i-1][j]>Image_Binary_Threshold)
				zero_one_num++;
			else
				zero_one_num--;
			
			if(image_buff[i][j]>Image_Binary_Threshold)
				zero_one_num++;
			else
				zero_one_num--;
			
			if(image_buff[i+1][j]>Image_Binary_Threshold)
				zero_one_num++;
			else
				zero_one_num--;
			
			if(image_buff[i+2][j]>Image_Binary_Threshold)
				zero_one_num++;
			else
				zero_one_num--;
			
			if(zero_one_num==0)
			{
				image_border[count]=i;
				break;
			}
		}
//		if(image_border[count]<8)
//			image_border[count]=8;
		
		count++;
	}
	
	//通过LCD显示图像数据
	LCD_brush_color=GREEN;
	LCD_DrawRectangle(249,129,801,195);
			
	LCD_Set_Window(250,130,800,194);
	LCD_WriteRAM_Prepare();
	
	for(i=0;i<64;i++)
	{
		for(j=0;j<550;j++)
		{
			if(j==start_location || j==start_location-length)
			{
				LCD_Write_Data(RED);
			}
			else
			{
				if(image_buff[i][j]>Image_Binary_Threshold)
					LCD_Write_Data(0xFFFE);
				else
					LCD_Write_Data(0x0001);
			}
		}
	}
	LCD_brush_color=RED;
	for(i=0;i<length;i++)
	{
		LCD_DrawPoint(start_location-i+250,image_border[i]+130);
	}
		
	return 0;
}



uint32_t Storage_Section_Check(uint32_t address)
{
	uint8_t success_flag=0;
	uint16_t i;
	
	for(i=0;i<10240;i++)
		camera_buff[0][i]=0xAA;
	
	while(!success_flag)
	{
		while(SD_OK!=SD_Erase(address,address+409600));//
		success_flag=1;
		for(i=0;i<620;i+=20)//
		{
			my_delayms(5);
			if(SD_OK!=SD_WriteMultiBlocks(camera_buff[0],address+512*i,512,20))//
			{
				SD_Init();
				success_flag=0;
				address+=409600;
				break;
			}
		}
	}
	return address;
}



//0：分配指定编号
//1：删除指定编号
void Operate_User_serialnum(uint8_t opt,uint16_t serialnum)
{
	if(SD_OK!=SD_ReadMultiBlocks(camera_buff[0],0,512,3))
		return;
	my_delayms(2);
	SD_Erase(0,1536);
	my_delayms(2);
	if(opt)
		camera_buff[0][serialnum]=1;
	else
		camera_buff[0][serialnum]=0;
	SD_WriteMultiBlocks(camera_buff[0],0,512,3);
}

//返回1-9999中未使用的最小的编号
uint16_t Allot_User_serialnum(void)
{
	uint16_t i;
	if(SD_OK!=SD_ReadMultiBlocks(camera_buff[0],0,512,3))
		return 0;
	for(i=1;i<9999;i++)
	{
		if(camera_buff[0][i])
			return i;
	}
	return 0;
}

void Image_Info_Save(uint32_t address,uint8_t* pbuff,uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
		camera_buff[0][i]=*pbuff++;
	}
	
	address+=307200;
	while(SD_OK!=SD_Erase(address,address+512));
	my_delayms(5);
	SD_WriteBlock(camera_buff[0],address,512);
}

void Image_Info_Read(uint32_t address,uint8_t* pbuff,uint8_t len)
{
	uint8_t i;
	
	address+=307200;
	
	SD_ReadBlock(camera_buff[0],address,512);
	
	for(i=0;i<len;i++)
	{
		*pbuff++=camera_buff[0][i];
	}
}


void Incise_Key(void)
{
	uint16_t i,j;
	int16_t k;
	uint16_t length;
	Platform_initial_position();//初始化平台中央
	Platform_work_position();
	if(key_tepe==1)
		length=350;
	else if(key_tepe==2)
		length=265;
	//开始切割
 	k=(54-image_border[0])*74;//
	for(i=0;i<length;i++)
	{
		if(k>0)
		{
			for(j=0;j<k;j++)
				StepMotor_Move(MOVE_L,500);
		}
		else if(k<0)
		{
			k=-k;
			for(j=0;j<k;j++)
				StepMotor_Move(MOVE_R,500);
		}
		
		if(i<90||i>260)
		{
			for(j=0;j<72;j++)
				StepMotor_Move(MOVE_B,300);
		}
		else
		{
			for(j=0;j<73;j++)
				StepMotor_Move(MOVE_B,300);
		}
		
		k=(image_border[i]-image_border[i+1])*75;
		if(i==length-1)
			k=0;
		
		//画出当前位置
		LCD_brush_color=MAGENTA;
		LCD_DrawPoint(start_location-i+250,image_border[i]+129);
		LCD_DrawPoint(start_location-i+250,image_border[i]+130);
		LCD_DrawPoint(start_location-i+250,image_border[i]+131);
		LCD_DrawPoint(start_location-i+250,image_border[i]+132);
	}
	for(j=0;j<1000;j++)
			StepMotor_Move(MOVE_R,300);
	MOTOR_OFF();
	Platform_initial_position();
}

void UART_Send_Image(void)
{
	uint16_t i,j,k;
	UART_Send_Byte(255);
	
//	for(j=0;j<600;j+=20)
//	{
//		k=5;
//		while(--k)
//		{
//			if(SD_OK==SD_ReadMultiBlocks(camera_buff[0],512*j,512,20))
//				break;
//		}
//		if(k==0)
//			while(1);
//			
//		for(i=0;i<10240;i+=4)
//		{
//			if(!(i%640))
//			{
//				i+=640*3;
//				if(i>10240)
//					break;
//			}
//			
//			if(camera_buff[0][i]==0)
//				camera_buff[0][i]=1;
//			else if(camera_buff[0][i]==255)
//				camera_buff[0][i]=254;
//			
//			UART_Send_Byte(camera_buff[0][i]);
//		}
//	}

	for(j=0;j<600;j+=10)
	{
		k=5;
		while(--k)
		{
			if(SD_OK==SD_ReadMultiBlocks(camera_buff[0],512*j,512,10))
				break;
		}
		if(k==0)
			while(1);
			
		for(i=0;i<5120;i++)
		{
			if(camera_buff[0][i]==0)
				camera_buff[0][i]=1;
			else if(camera_buff[0][i]==255)
				camera_buff[0][i]=254;
			
			UART_Send_Byte(camera_buff[0][i]);
		}
	}
}


