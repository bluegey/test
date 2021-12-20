/*************************************ͷ�ļ�******************************************/
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

/*************************************�궨��******************************************/
#define Image_Binary_Threshold	145//��ֵ����ֵ

#define	SD_INIT_ERROR				1
#define	SD_WRITE_ERROR			2
#define	SD_READ_ERROR				3
#define	SD_ERASE_ERROR			4
#define	CAMERA_INIT_ERROR		5

/************************************ȫ�ֱ���******************************************/
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

/************************************��������******************************************/
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
		if(menuitem==1)//����Կ��
		{
			while(1)
			{
				menuitem=Display_new_key();
				if(menuitem==0)
					break;
			}
		}
		else if(menuitem==2)//���Կ��
		{
			while(1)
			{
				menuitem=Display_old_key();
				if(menuitem==0)
					break;
			}
		}
		else if(menuitem==3)//ϵͳ����
		{
			while(1)
			{
				menuitem=Display_system_set();
				if(menuitem==1)//�ƹ����
				{
					Display_brightness_set();
				}
				else if(menuitem==2)//����У׼
				{
					tp_dev.adjust();
				}
				else if(menuitem==3)//������Ŀ
				{
					Display_about_project();
				}
				else if(menuitem==0)//����
				{
					break;
				}
			}
		}
		else if(menuitem==4)//����
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
		LCD_Display_Message(204,200,"�������������߹ػ����SD��");
		
		LCD_Display_Message(204,300,"����");
		LCD_Display_Message(556,300,"�ػ�");
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(199,295,273,337);
		LCD_DrawRectangle(200,296,272,336);//�����߿�
		LCD_DrawRectangle(551,295,625,337);
		LCD_DrawRectangle(552,296,624,336);//�ػ��߿�
		
		TP_IRQ_ENABLE();//��������
		while(1)
		{
			if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
			{		
				tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
				
				if(TP_Area_Press_Judge(200,295,270,335))//��������������
				{
					LCD_Clear(BLACK);
					NVIC_SystemReset();
				}
				else if(TP_Area_Press_Judge(555,295,625,335))//�ػ�����������
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
		LCD_Display_Message(300,150,"����ͷ��ʼ��ʧ�ܣ�");
		LCD_Display_Message(204,200,"�������������߹ػ��������ͷ");
		
		LCD_Display_Message(204,300,"����");
		LCD_Display_Message(588,300,"�ػ�");
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(199,295,273,337);
		LCD_DrawRectangle(200,296,272,336);//�����߿�
		LCD_DrawRectangle(582,295,657,337);
		LCD_DrawRectangle(583,296,656,336);//�ػ��߿�
		
		TP_IRQ_ENABLE();//��������
		while(1)
		{
			if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
			{		
				tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
				
				if(TP_Area_Press_Judge(200,295,270,335))//��������������
				{
					LCD_Clear(BLACK);
					NVIC_SystemReset();
				}
				else if(TP_Area_Press_Judge(585,295,655,335))//�ػ�����������
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
		LCD_Display_Message(316,150,"SD��д��ʧ�ܣ�");
		LCD_Display_Message(204,200,"�������������߹ػ����SD��");
		
		LCD_Display_Message(204,300," ");
		LCD_Display_Message(556,300,"�ػ�");
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(199,295,273,337);
		LCD_DrawRectangle(200,296,272,336);//�����߿�
		LCD_DrawRectangle(551,295,625,337);
		LCD_DrawRectangle(552,296,624,336);//�ػ��߿�
		
		TP_IRQ_ENABLE();//��������
		while(1)
		{
			if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
			{		
				tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
				
				if(TP_Area_Press_Judge(200,295,270,335))//��������������
				{
					LCD_Clear(BLACK);
					NVIC_SystemReset();
				}
				else if(TP_Area_Press_Judge(555,295,625,335))//�ػ�����������
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
		LCD_Display_Message(316,150,"SD����ȡʧ�ܣ�");
		LCD_Display_Message(204,200,"�������������߹ػ����SD��");
		
		LCD_Display_Message(204,300,"����");
		LCD_Display_Message(556,300,"�ػ�");
		
		LCD_brush_color=BLUE;
		LCD_DrawRectangle(199,295,273,337);
		LCD_DrawRectangle(200,296,272,336);//�����߿�
		LCD_DrawRectangle(551,295,625,337);
		LCD_DrawRectangle(552,296,624,336);//�ػ��߿�
		
		TP_IRQ_ENABLE();//��������
		while(1)
		{
			if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
			{		
				tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
				
				if(TP_Area_Press_Judge(200,295,270,335))//��������������
				{
					LCD_Clear(BLACK);
					NVIC_SystemReset();
				}
				else if(TP_Area_Press_Judge(555,295,625,335))//�ػ�����������
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
	LCD_Display_Message(171,30,"ȫ �� �� �� �� Կ �� �� ȡ װ ��");
	
	LCD_Fill(333,104,521,196,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,134,"����Կ��");
	
	LCD_Fill(333,204,521,296,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,234,"���Կ��");
	
	LCD_Fill(333,304,521,396,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,334,"ϵͳ����");
	
	LCD_Fill(0,428,84,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(10,438,"����");
	
	LCD_Fill(770,428,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(780,438,"�ػ�");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_string_font=ASCII_1224;
	LCD_Display_Message(193,446,"������Ϣ����ѧԺ---����ѵ������ 2015.12");
	
	TP_IRQ_ENABLE();//��������
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{		
			tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
			
			if(TP_Area_Press_Judge(333,104,521,196))//����Կ�װ���������
			{
				temp=1;
				break;
			}
			else if(TP_Area_Press_Judge(333,204,521,296))//���Կ�װ���������
			{
				temp=2;
				break;
			}
			else if(TP_Area_Press_Judge(333,304,521,396))//ϵͳ���ð���������
			{
				temp=3;
				break;
			}
			else if(TP_Area_Press_Judge(0,436,68,480))//��������������
			{
				temp=4;
				break;
			}
			else if(TP_Area_Press_Judge(786,436,854,480))//�ػ�����������
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
	LCD_Display_Message(50,30,"ϵ  ͳ  ��  ��");
	
	LCD_Fill(333,104,521,196,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,134,"�ƹ����");
	
	LCD_Fill(333,204,521,296,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,234,"����У׼");
	
	LCD_Fill(333,304,521,396,CYAN);
	LCD_board_color=CYAN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(363,334,"������Ŀ");
	
	LCD_Fill(786,436,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(796,446,"����");
	
	TP_IRQ_ENABLE();//��������
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{		
			tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
			
			if(TP_Area_Press_Judge(333,104,521,196))//�ƹ���ڰ���������
			{
				temp=1;
				break;
			}
			else if(TP_Area_Press_Judge(333,204,521,296))//����У׼����������
			{
				temp=2;
				break;
			}
			else if(TP_Area_Press_Judge(333,304,521,396))//������Ŀ����������
			{
				temp=3;
				break;
			}
			else if(TP_Area_Press_Judge(786,436,854,480))//���ذ���������
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
	LCD_Display_Message(50,30,"�� �� Կ ��");
	
	LCD_board_color=WHITE;
	LCD_brush_color=MAGENTA;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,120,"���� 1 :");
	LCD_Display_Chinese(30,160,"ͼ��ɼ�����");
	LCD_Display_Message(400,140,"���ڲɼ�ͼ��...");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,255,"���� 2 :");
	LCD_Display_Chinese(30,295,"��ʼ��ȡԿ��");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,390,"���� 3 :");
	LCD_Display_Chinese(30,430,"����Կ������");
	
	
	//��һ��ͼ��ɼ�����ͼ����ʧ�ܺ�ѯ���Ƿ���Ҫ���²ɼ���Ĭ�ϣ���ͼ����ɹ���ʼ�����иĬ�ϣ�
	AT24CXX_Read(20,(uint8_t*)(&storage_info),10);//��ȡ���һ��ͼƬ��ַ
	add=Storage_Section_Check(storage_info.last+409600);//
	collect_success_flag=0;
	while(!collect_success_flag)
	{
		Image_Collect(add);//�ɼ�ͼ��
		if(Image_Process(add))//ͼ����ʧ�ܣ�ѯ���Ƿ���Ҫ���²ɼ�
		{
			count_1=200;
			
			LCD_board_color=WHITE;
			LCD_brush_color=RED;
			LCD_chinese_font=Chinese_3232;
			LCD_string_font=ASCII_1632;
			LCD_Display_Chinese(397,250,	   "ͼ��ɼ�����ʧ��");
			LCD_Display_Chinese(317,290,"����Կ�׺����²ɼ��򷵻�");
			
			LCD_Display_Chinese(678,410,"����");
			LCD_Display_Message(308,410,"����(9s)");
			
			LCD_DrawRectangle(295,230,755,460);
			LCD_DrawRectangle(303,400,441,452);
			LCD_DrawRectangle(673,400,747,452);
			
			TP_IRQ_ENABLE();
			while(1)
			{
				if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
				{		
					tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
					
					if(TP_Area_Press_Judge(303,400,441,452))//���԰���������
					{
						TP_IRQ_DISABLE();
						LCD_Fill(240,120,810,470,WHITE);
						break;
					}
					else if(TP_Area_Press_Judge(673,400,747,452))//���ذ���������
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
			LCD_Display_Chinese(397,250,	  "ͼ��ɼ�����ɹ�");
			LCD_Display_Chinese(323,290,"��ȷ���Ƿ�ʹ��ͼ��򷵻�");
			
			LCD_Display_Chinese(678,410,"����");
			LCD_Display_Message(308,410,"ȷ��(5s)");
			
			LCD_DrawRectangle(295,230,755,460);
			LCD_DrawRectangle(303,400,441,452);
			LCD_DrawRectangle(673,400,747,452);
			
			TP_IRQ_ENABLE();
			while(1)
			{
				if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
				{		
					tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
					
					if(TP_Area_Press_Judge(303,400,441,452))//ȷ������������
					{
						TP_IRQ_DISABLE();
//						LCD_Fill(240,120,810,470,WHITE);
						collect_success_flag=1;
						break;
					}
					else if(TP_Area_Press_Judge(673,400,747,452))//���ذ���������
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
	
	//�ڶ����ȼ��ƽ̨�Ƿ��ڰ�ȫλ�ã�Ȼ��ƽ̨�ƶ�������λ�ã���������鿴�Ƿ���Ҫ΢����Ĭ�ϲ���Ҫ�������ʼ�����и�и���ɺ�ָ�ƽ̨����ȫλ��
	LCD_board_color=WHITE;
	LCD_brush_color=LGRAY;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,120,"���� 1 :");
	LCD_Display_Chinese(30,160,"ͼ��ɼ�����");
	
	LCD_board_color=WHITE;
	LCD_brush_color=MAGENTA;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,255,"���� 2 :");
	LCD_Display_Chinese(30,295,"��ʼ��ȡԿ��");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,390,"���� 3 :");
	LCD_Display_Chinese(30,430,"����Կ������");
	
	LCD_Fill(290,225,760,465,WHITE);
	LCD_brush_color=RED;
	LCD_board_color=WHITE;
	LCD_DrawRectangle(295,230,755,460);
	LCD_Display_Message(333,329,"�����и�Կ��,��ע�ⰲȫ!");
	
	Incise_Key();
	
	//������ѯ���Ƿ񱣴��û����ݣ�Ĭ�ϲ�����
	LCD_board_color=WHITE;
	LCD_brush_color=LGRAY;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,120,"���� 1 :");
	LCD_Display_Chinese(30,160,"ͼ��ɼ�����");
	
	LCD_board_color=WHITE;
	LCD_brush_color=LGRAY;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,255,"���� 2 :");
	LCD_Display_Chinese(30,295,"��ʼ��ȡԿ��");
	
	LCD_board_color=WHITE;
	LCD_brush_color=MAGENTA;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,390,"���� 3 :");
	LCD_Display_Chinese(30,430,"����Կ������");	
	
	LCD_Fill(240,100,840,470,WHITE);
	
	LCD_board_color=WHITE;
	LCD_brush_color=RED;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(301,150,"��ȡ��Կ��, �Ƿ񱣴��û�����");
	LCD_Display_Message(413,200,"�û����: ");
	num_1=Allot_User_serialnum();
	LCD_Display_Format(587,200,"%4d",num_1);
	
	LCD_Display_Message(285,393,"����");
	LCD_Display_Message(637,393,"ȡ��(9s)");
	
	LCD_DrawRectangle(270,130,780,440);
	LCD_DrawRectangle(280,388,354,430);
	LCD_DrawRectangle(632,388,770,430);
	
	TP_IRQ_ENABLE();
	count_1=200;
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{		
			tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
			
			if(TP_Area_Press_Judge(280,388,354,430))//���水��������
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
			else if(TP_Area_Press_Judge(632,388,770,430))//ȡ������������
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
	LCD_Display_Message(50,30,"�� �� Կ ��");
	
	LCD_Fill(786,436,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(796,446,"����");
	
	AT24CXX_Read(20,(uint8_t*)(&storage_info),10);

	if(storage_info.total==0)//���û�����
	{
		LCD_board_color=WHITE;
		LCD_brush_color=RED;
		LCD_string_font=ASCII_1632;
		LCD_chinese_font=Chinese_3232;
		LCD_Display_Chinese(331,230,"û���û�����");
		LCD_DrawRectangle(250,180,604,312);
		
		TP_IRQ_ENABLE();
		while(1)
		{
			if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
			{		
				tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
				
				if(TP_Area_Press_Judge(786,436,854,480))//���ذ���������
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
		LCD_Display_Chinese(64,114,"��һҳ");
		LCD_Display_Chinese(64,434,"��һҳ");
		LCD_DrawRectangle(20,107,180,145);
		LCD_DrawRectangle(20,427,180,465);
		
		LCD_board_color=WHITE;
		LCD_brush_color=DGREEN;
		LCD_chinese_font=Chinese_3232;
		LCD_string_font=ASCII_1632;
		LCD_Display_Chinese(429,250,"ʹ��ģ����ȡ");
		LCD_Display_Chinese(429,360,"ɾ���û�����");
		LCD_DrawRectangle(350,240,700,292);
		LCD_DrawRectangle(350,350,700,402);
		
		LCD_board_color=CYAN;
		LCD_brush_color=BLACK;
		LCD_chinese_font=Chinese_3232;
		LCD_string_font=ASCII_1632;
		LCD_Display_Message(20,150,"���:");
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
			LCD_Display_Message(20,150+40*i,"���:");
			LCD_Display_Format(100,150+40*i,"%c%d",image_info_temp[i].type,image_info_temp[i].serial_num);
			if(i==6)
				break;
		}
		
	}
	my_delayms(1000);
	TP_IRQ_ENABLE();
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{		
			TP_IRQ_DISABLE();
			tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
			if(TP_Area_Press_Judge(786,436,854,480))//���ذ���������
			{
				return 0;
			}
			else if(TP_Area_Press_Judge(350,240,700,292))//��ȡ����������
			{
				LCD_Fill(290,225,760,465,WHITE);
				LCD_chinese_font=Chinese_3232;
				LCD_string_font=ASCII_1632;
				LCD_brush_color=RED;
				LCD_board_color=WHITE;
				LCD_DrawRectangle(295,230,755,460);
				LCD_Display_Message(333,329,"�����и�Կ��,��ע�ⰲȫ!");
	
				Incise_Key();
				return 1;
			}
			else if(TP_Area_Press_Judge(350,350,700,402))//ɾ������������
			{
				if((page-1)*7+select_num==1)//ɾ����һ������
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
				else if((page-1)*7+select_num==storage_info.total)//ɾ�����һ������
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
			else if(TP_Area_Press_Judge(20,107,180,145))//��һҳ����������
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
					LCD_Display_Message(20,150,"���:");
					LCD_Display_Format(100,150,"%c%d",image_info_temp[0].type,image_info_temp[0].serial_num);
					Image_Process(image_info_temp[0].self);
					
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					
					for(ii=1;ii<7;ii++)
					{
						LCD_Display_Message(20,150+40*ii,"���:");
						LCD_Display_Format(100,150+40*ii,"%c%d",image_info_temp[ii].type,image_info_temp[ii].serial_num);
					}
				}
			}
			else if(TP_Area_Press_Judge(20,427,180,465))//��һҳ����������
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
					LCD_Display_Message(20,150,"���:");
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
						LCD_Display_Message(20,150+40*jj,"���:");
						LCD_Display_Format(100,150+40*jj,"%c%d",image_info_temp[jj].type,image_info_temp[jj].serial_num);
						if(jj==6)
							break;
					}
				}
			}
			else if(TP_Area_Press_Judge(20,146,180,186))//1����������
			{
				if((((page-1)*7+1)<=storage_info.total)&&(select_num!=1))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=1;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,186,180,226))//2����������
			{
				if((((page-1)*7+2)<=storage_info.total)&&(select_num!=2))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=2;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,226,180,266))//3����������
			{
				if((((page-1)*7+3)<=storage_info.total)&&(select_num!=3))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=3;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,266,180,306))//4����������
			{
				if((((page-1)*7+4)<=storage_info.total)&&(select_num!=4))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=4;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,306,180,346))//5����������
			{
				if((((page-1)*7+5)<=storage_info.total)&&(select_num!=5))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=5;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,346,180,386))//6����������
			{
				if((((page-1)*7+6)<=storage_info.total)&&(select_num!=6))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=6;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					Image_Process(image_info_temp[select_num-1].self);
				}
			}
			else if(TP_Area_Press_Judge(20,386,180,426))//7����������
			{
				if((((page-1)*7+7)<=storage_info.total)&&(select_num!=7))
				{
					LCD_board_color=WHITE;
					LCD_brush_color=BLACK;
					LCD_chinese_font=Chinese_3232;
					LCD_string_font=ASCII_1632;
					LCD_Display_Message(20,110+40*select_num,"���:");
					LCD_Display_Format(100,110+40*select_num,"%c%d",image_info_temp[select_num-1].type,image_info_temp[select_num-1].serial_num);
					
					select_num=7;
					LCD_board_color=CYAN;
					LCD_brush_color=BLACK;
					LCD_Display_Message(20,110+40*select_num,"���:");
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
	LCD_Display_Message(50,30,"�� �� �� Ŀ");
	
	LCD_board_color=GREEN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	
	LCD_Display_Message(50,120,"��Ŀ����:ȫ�Զ�����Կ����ȡװ��");
	LCD_Display_Message(50,180,"��Ŀ���:���Ҽ���ѧ�����´�ҵѵ���ƻ���Ŀ");
	LCD_Display_Message(50,240,"��Ŀ��Ա:����,۳����,����,������,����");
	LCD_Display_Message(50,300,"ָ����ʦ:��־ȫ");
	
	LCD_chinese_font=Chinese_2424;
	LCD_string_font=ASCII_1224;
	LCD_Display_Message(211,380,"***������Ϣ����ѧԺ--����ѵ������***");
	
	LCD_Fill(786,436,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(796,446,"����");
	
//	LCD_DrawRectangle(377,50,477,150);
//	LCD_DrawRectangle(377,330,477,430);
//	LCD_DrawRectangle(227,190,327,290);
//	LCD_DrawRectangle(527,190,627,290);
	
	TP_IRQ_ENABLE();//��������
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{	
			TP_IRQ_DISABLE();
			tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
			if(TP_Area_Press_Judge(786,436,854,480))//���ذ���������
			{
				break;
			}
			else if(TP_Area_Press_Judge(377,50,477,150))//���ذ���������
			{
				value=1;
			}
			else if(TP_Area_Press_Judge(527,190,627,290))//���ذ���������
			{
				if(value==1)
					value=2;
			}
			else if(TP_Area_Press_Judge(377,330,477,430))//���ذ���������
			{
				if(value==2)
					value=3;
			}
			else if(TP_Area_Press_Judge(227,190,327,290))//���ذ���������
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
	LCD_Display_Message(50,30,"�� �� �� ��");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(760,100,"��");
	LCD_Display_Chinese(760,396,"��");
	
	LCD_Fill(786,436,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(796,446,"����");

	LCD_Fill(720,100,750,420,LGRAY);
	LCD_Fill(730,110,740,410,0x4208);
	LCD_Fill(725,255,745,265,DGRAY);
	
	LCD_brush_color=GREEN;
	LCD_DrawRectangle(19,99,661,421);
	
	
	Image_Collect((uint32_t)409600*9000);
	LCD_Show_Image((uint32_t)409600*9000);
	TP_IRQ_ENABLE();//��������
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{		
			TP_IRQ_DISABLE();
			tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
			if(TP_Area_Press_Judge(786,436,854,480))//���ذ���������
			{
				break;
			}
			else if(TP_Area_Press_Judge(710,110,760,410))//�ƹ���������
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
	LCD_Display_Message(50,30,"��  ��");
	
	LCD_Fill(786,436,854,480,RED);
	LCD_board_color=RED;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(796,446,"����");
	
	LCD_board_color=WHITE;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_string_font=ASCII_1632;
	LCD_Display_Message(20,150,"����Կ��:�û��״�ʹ�ñ��豸��ȡԿ��");
	LCD_Display_Message(20,250,"���Կ��:�û�ʹ�ñ����������ȡԿ��");
	LCD_Display_Message(20,350,"ϵͳ����:�������ݲɼ�ʱ�ƹ�ǿ��,����У׼�Ȳ���");
	
	TP_IRQ_ENABLE();//��������
	while(1)
	{
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{		
			tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
			
			if(TP_Area_Press_Judge(786,436,854,480))//���ذ���������
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
//�������ж�	
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
	
	//��SD���ж�ȡ��������
	//�ж��ǲ��Ǵ�Կ��
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
	
	//Ѱ��Կ����ʼλ��
	//Կ����ʼλ��һ���ڵ�40-63�У���420-520��
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
		key_tepe=2;//���Ǵ�Կ�ף������ж��ǲ���СԿ��
		length=265;
	}
	
	//�ж��ǲ���СԿ��
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
		
		//Ѱ��Կ����ʼλ��
		//Կ����ʼλ��һ���ڵ�40-63�У���420-520��
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
	
	//����ʼλ�ÿ�ʼ��Ѱ�ҳݱ�Ե
	//Կ���ܳ��ȴ�Լ350�����ص�
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
	
	//ͨ��LCD��ʾͼ������
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



//0������ָ�����
//1��ɾ��ָ�����
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

//����1-9999��δʹ�õ���С�ı��
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
	Platform_initial_position();//��ʼ��ƽ̨����
	Platform_work_position();
	if(key_tepe==1)
		length=350;
	else if(key_tepe==2)
		length=265;
	//��ʼ�и�
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
		
		//������ǰλ��
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


