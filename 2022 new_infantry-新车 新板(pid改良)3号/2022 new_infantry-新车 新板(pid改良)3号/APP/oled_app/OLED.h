#ifndef __OLED_H
#define __OLED_H

#include "sys.h"

//IICЭ�飺0
//SPIЭ�飺1
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

#define OLED_RES_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_10)// USART3_TX(��������)---PB10
#define OLED_RES_Set() GPIO_SetBits(GPIOB,GPIO_Pin_10)

#define OLED_DC_Clr()  GPIO_ResetBits(GPIOB,GPIO_Pin_11)//DC/USART3_RX(��������)---PB11
#define OLED_DC_Set()  GPIO_SetBits(GPIOB,GPIO_Pin_11)
 		     
#define OLED_CS_Clr()  GPIO_ResetBits(GPIOB,GPIO_Pin_12)// SPI2_NSS --- PB12
#define OLED_CS_Set()  GPIO_SetBits(GPIOB,GPIO_Pin_12)

#else
#include "IIC.h"
#endif

//#include "UI.h"

void OLED_DBMP(int x0, int y0, int bmpx, int bmpy, const char *bmp);	//��ʾλͼ
void OLED_FSize(int size);												//���������С
void OLED_PStr(int x0, int y0, const char *str);						//��ӡ�ַ���
void OLED_PCh(int x0, int y0, char ch);									//��ӡ�����ַ�
void OLED_printf (int x0, int y0, const char *str, ...);					//��ӡ����
void OLED_Display(void);												//��ʾ��������
void OLED_DClean(void);
void OLED_Clean(void);													//��ջ���
void OLED_Init(void);													//��ʼ��

#endif

