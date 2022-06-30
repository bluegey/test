#ifndef __OLED_H
#define __OLED_H

#include "sys.h"

//IIC协议：0
//SPI协议：1
#define OLED_SEL_BUS	1

#if	OLED_SEL_BUS
//#include "SPI.h"

//#define OLED_RES_GPIO_PIN	GPIO_Pin_8
//#define OLED_RES_GPIO_PORT	GPIOD
//#define OLED_RES_GPIO_CLK	RCC_APB2Periph_GPIOD

//#define OLED_DC_GPIO_PIN	GPIO_Pin_9
//#define OLED_DC_GPIO_PORT	GPIOD
//#define OLED_DC_GPIO_CLK	RCC_APB2Periph_GPIOD

//#define OLED_CS_GPIO_PIN	GPIO_Pin_10
//#define OLED_CS_GPIO_PORT	GPIOD
//#define OLED_CS_GPIO_CLK	RCC_APB2Periph_GPIOD

#define OLED_SCL_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_13)//SCL /sck /PB13  O
#define OLED_SCL_Set() GPIO_SetBits(GPIOB,GPIO_Pin_13)

#define OLED_SDA_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_15)//SDA/mosi /PB15
#define OLED_SDA_Set() GPIO_SetBits(GPIOB,GPIO_Pin_15)

#define OLED_RES_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_10)// USART3_TX(步兵板上)---PB10
#define OLED_RES_Set() GPIO_SetBits(GPIOB,GPIO_Pin_10)

#define OLED_DC_Clr()  GPIO_ResetBits(GPIOB,GPIO_Pin_11)//DC/USART3_RX(步兵板上)---PB11
#define OLED_DC_Set()  GPIO_SetBits(GPIOB,GPIO_Pin_11)
 		     
#define OLED_CS_Clr()  GPIO_ResetBits(GPIOB,GPIO_Pin_12)// SPI2_NSS --- PB12
#define OLED_CS_Set()  GPIO_SetBits(GPIOB,GPIO_Pin_12)

#else
#include "IIC.h"
#endif

//#include "UI.h"

void OLED_DBMP(int x0, int y0, int bmpx, int bmpy, const char *bmp);	//显示位图
void OLED_FSize(int size);												//设置字体大小
void OLED_PStr(int x0, int y0, const char *str);						//打印字符串
void OLED_PCh(int x0, int y0, char ch);									//打印单个字符
void OLED_printf (int x0, int y0, const char *str, ...);					//打印数据
void OLED_Display(void);												//显示缓存数据
void OLED_DClean(void);
void OLED_Clean(void);													//清空缓存
void OLED_Init(void);													//初始化

#endif

