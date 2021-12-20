#ifndef	_LCD_H_	
#define	_LCD_H_

#include "stm32f10x.h"
#include "GT30L32S4W.h"

#define   BLACK                0x0020                // ��ɫ��    0,   0,   0 //
#define   BLUE                 0x001F                // ��ɫ��    0,   0, 255 //
#define   GREEN                0x07E0                // ��ɫ��    0, 255,   0 //
#define   CYAN                 0x07FF                // ��ɫ��    0, 255, 255 //
#define   RED                  0xF800                // ��ɫ��  255,   0,   0 //
#define   MAGENTA              0xF81F                // Ʒ�죺  255,   0, 255 //
#define   YELLOW               0xFFE0                // ��ɫ��  255, 255, 0   //
#define   WHITE                0xFFDF                // ��ɫ��  255, 255, 255 //
#define   NAVY                 0x000F                // ����ɫ��  0,   0, 128 //
#define   DGREEN               0x03E0                // ����ɫ��  0, 128,   0 //
#define   DCYAN                0x03EF                // ����ɫ��  0, 128, 128 //
#define   MAROON               0x7800                // ���ɫ��128,   0,   0 //
#define   PURPLE               0x780F                // ��ɫ��  128,   0, 128 //
#define   OLIVE                0x7BE0                // ����̣�128, 128,   0 //
#define   LGRAY                0xC618                // �Ұ�ɫ��192, 192, 192 //
#define   DGRAY                0x7BEF                // ���ɫ��128, 128, 128 //

//ɨ�跽����
#define L2R_U2D  0 //������,���ϵ���
#define L2R_D2U  1 //������,���µ���
#define R2L_U2D  2 //���ҵ���,���ϵ���
#define R2L_D2U  3 //���ҵ���,���µ���

#define U2D_L2R  4 //���ϵ���,������
#define U2D_R2L  5 //���ϵ���,���ҵ���
#define D2U_L2R  6 //���µ���,������
#define D2U_R2L  7 //���µ���,���ҵ���

#define DFT_SCAN_DIR  U2D_R2L  //Ĭ�ϵ�ɨ�跽��

extern uint16_t LCD_brush_color;
extern uint16_t LCD_board_color;
extern uint8_t	LCD_string_font;
extern uint8_t	LCD_chinese_font;
//LCD��Ҫ������
typedef struct  
{										    
	u16 width;			//LCD ���
	u16 height;			//LCD �߶�
	u16 id;				//LCD ID
	u8  dir;			//���������������ƣ�0��������1��������	
	u16	wramcmd;		//��ʼдgramָ��
	u16 setxcmd;		//����x����ָ��
	u16 setycmd;		//����y����ָ�� 
}_lcd_dev; 

extern _lcd_dev lcddev;

void LCD_Init(void);
void LCD_Scan_Dir(uint8_t dir);
void LCD_Display_Dir(uint8_t dir);
void LCD_Display_On(void);
void LCD_Display_Off(void);
void LCD_Write_Data(uint16_t DATA);
void LCD_WriteRAM_Prepare(void);
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void LCD_DrawPoint(uint16_t x,uint16_t y);
void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
void LCD_Set_Window(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey);
void LCD_Clear(uint16_t color);
void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r);

void LCD_Display_Font(uint16_t x,uint16_t y,uint8_t type,uint8_t* GBCode);
void LCD_Display_Chinese(uint16_t x,uint16_t y,uint8_t* GBCode);
void LCD_Display_String(uint16_t x,uint16_t y,uint8_t* GBCode);
void LCD_Display_Format(uint16_t x,uint16_t y,const char *fmt,...);
void LCD_Display_Message(uint16_t x,uint16_t y,uint8_t* GBCode);
	
#endif
