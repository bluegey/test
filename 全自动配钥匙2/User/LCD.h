#ifndef	_LCD_H_	
#define	_LCD_H_

#include "stm32f10x.h"
#include "GT30L32S4W.h"

#define   BLACK                0x0020                // 黑色：    0,   0,   0 //
#define   BLUE                 0x001F                // 蓝色：    0,   0, 255 //
#define   GREEN                0x07E0                // 绿色：    0, 255,   0 //
#define   CYAN                 0x07FF                // 青色：    0, 255, 255 //
#define   RED                  0xF800                // 红色：  255,   0,   0 //
#define   MAGENTA              0xF81F                // 品红：  255,   0, 255 //
#define   YELLOW               0xFFE0                // 黄色：  255, 255, 0   //
#define   WHITE                0xFFDF                // 白色：  255, 255, 255 //
#define   NAVY                 0x000F                // 深蓝色：  0,   0, 128 //
#define   DGREEN               0x03E0                // 深绿色：  0, 128,   0 //
#define   DCYAN                0x03EF                // 深青色：  0, 128, 128 //
#define   MAROON               0x7800                // 深红色：128,   0,   0 //
#define   PURPLE               0x780F                // 紫色：  128,   0, 128 //
#define   OLIVE                0x7BE0                // 橄榄绿：128, 128,   0 //
#define   LGRAY                0xC618                // 灰白色：192, 192, 192 //
#define   DGRAY                0x7BEF                // 深灰色：128, 128, 128 //

//扫描方向定义
#define L2R_U2D  0 //从左到右,从上到下
#define L2R_D2U  1 //从左到右,从下到上
#define R2L_U2D  2 //从右到左,从上到下
#define R2L_D2U  3 //从右到左,从下到上

#define U2D_L2R  4 //从上到下,从左到右
#define U2D_R2L  5 //从上到下,从右到左
#define D2U_L2R  6 //从下到上,从左到右
#define D2U_R2L  7 //从下到上,从右到左

#define DFT_SCAN_DIR  U2D_R2L  //默认的扫描方向

extern uint16_t LCD_brush_color;
extern uint16_t LCD_board_color;
extern uint8_t	LCD_string_font;
extern uint8_t	LCD_chinese_font;
//LCD重要参数集
typedef struct  
{										    
	u16 width;			//LCD 宽度
	u16 height;			//LCD 高度
	u16 id;				//LCD ID
	u8  dir;			//横屏还是竖屏控制：0，竖屏；1，横屏。	
	u16	wramcmd;		//开始写gram指令
	u16 setxcmd;		//设置x坐标指令
	u16 setycmd;		//设置y坐标指令 
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
