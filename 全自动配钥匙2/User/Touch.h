#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "stm32f10x.h"

extern uint8_t TP_enable_irq_flag;

#define TP_IRQ_ENABLE()		{TP_enable_irq_flag=1;EXTI_ClearITPendingBit(EXTI_Line1);NVIC_EnableIRQ(EXTI1_IRQn);}
#define TP_IRQ_DISABLE()	{TP_enable_irq_flag=0;NVIC_DisableIRQ(EXTI1_IRQn);}

#define TP_PRES_DOWN 0x80  		//触屏被按下	  
#define TP_CATH_PRES 0x40  		//有按键按下

//触摸屏控制器
typedef struct 
{
	u8 (*init)(void);			//初始化触摸屏控制器
	u8 (*scan)(u8);				//扫描触摸屏.0,屏幕扫描;1,物理坐标;	 
	void (*adjust)(void);	//触摸屏校准 
	u16 x; 		//当前坐标
	u16 y;		//电容屏有最多5组坐标,电阻屏则用x[0],y[0]代表:此次扫描时,触屏的坐标,用
						//x[4],y[4]存储第一次按下时的坐标. 
	u8  sta;	//笔的状态 
						//b7:按下1/松开0; 
	          //b6:0,没有按键按下;1,有按键按下. 
						//b5:保留
						//b4~b0:电容触摸屏按下的点数(0,表示未按下,1表示按下)
/////////////////////触摸屏校准参数(电容屏不需要校准)//////////////////////								
	float xfac;					
	float yfac;
	short xoff;
	short yoff;	   
//新增的参数,当触摸屏的左右上下完全颠倒时需要用到.
//b0:0,竖屏(适合左右为X坐标,上下为Y坐标的TP)
//   1,横屏(适合左右为Y坐标,上下为X坐标的TP) 
//b1~6:保留.
//b7:0,电阻屏
//   1,电容屏 
	u8 touchtype;
	u8 adjust_flag;
}_m_tp_dev;

extern _m_tp_dev tp_dev;	 	//触屏控制器在touch.c里面定义


void TP_Adjust(void);
uint8_t TP_Init(void);
uint8_t TP_Scan(uint8_t tp);
uint8_t TP_Area_Press_Judge(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey);


#endif
