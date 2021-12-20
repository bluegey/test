#ifndef _OLED_H_
#define _OLED_H_

/**************************************************************************************************
 *                                          INCLUDES
 **************************************************************************************************/
#include "stm32f10x.h"

/**************************************************************************************************
 *                                         CONSTANTS
 **************************************************************************************************/


/**************************************************************************************************
 *                                          MACROS
 **************************************************************************************************/


/**************************************************************************************************
 *                                         TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                     GLOBAL VARIABLES
 **************************************************************************************************/


/**************************************************************************************************
 *                                     FUNCTIONS - API
 **************************************************************************************************/
void OLED_Clear(void);
void OLED_Init(void);
void OLED_Checkerboard(void);
void OLED_Fill_RAM(unsigned char Data);
void OLED_Set_Start_Page(unsigned char dd);
void OLED_Set_Start_Column(unsigned char d);
void OLED_setXY(unsigned char X,unsigned char Y);
void OLED_Display_Char(unsigned char x,unsigned char y,char c);
void OLED_Display_String(unsigned char x,unsigned char y,char* str);
void OLED_Display_Format(unsigned char x,unsigned char y,const char* format,...);




void Sleep(unsigned char a);
void Deactivate_Scroll(void);
void Set_VCOMH(unsigned char dd);
void Set_Start_Line(unsigned char ddd);
void Set_Common_Remap(unsigned char dd);
void Set_Charge_Pump(unsigned char ddd);
void Set_Common_Config(unsigned char dd);
void Set_Display_Clock(unsigned char dd);
void Set_Segment_Remap(unsigned char ddd);
void Set_Display_Offset(unsigned char dd);
void Set_Display_On_Off(unsigned char dd);
void Set_Addressing_Mode(unsigned char d);
void Set_Entire_Display(unsigned char ddd);
void Set_Multiplex_Ratio(unsigned char ddd);
void Set_Inverse_Display(unsigned char ddd);
void Set_Precharge_Period(unsigned char dd);
void Set_Contrast_Control(unsigned char ddd);
void Set_Page_Address(unsigned char a, unsigned char b);
void Set_Column_Address(unsigned char a, unsigned char b);
void Fill_Block(unsigned char Data, unsigned char a, unsigned char b, unsigned char c, unsigned char d);
void Vertical_Scroll(unsigned char a, unsigned char b, unsigned char c, unsigned char d, unsigned char e);
void Horizontal_Scroll(unsigned char a, unsigned char b, unsigned char c, unsigned char dd, unsigned char ed);
void Show_Pattern(unsigned char *Data_Pointer, unsigned char a, unsigned char b, unsigned char c, unsigned char d);
void Continuous_Scroll(unsigned char a, unsigned char b, unsigned char c, unsigned char dd, unsigned char ed, unsigned char f, unsigned char g, unsigned char h);

/**************************************************************************************************
**************************************************************************************************/

#endif
