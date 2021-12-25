#ifndef _MYIIC_H
#define _MYIIC_H
#include "main.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F103开发板
//IIC驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2019/9/18
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
//IO方向设置
//#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}	//PB7输入模式
//#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;} 	//PB7输出模式

//IO操作
#define IIC_Pin_SCL		GPIO_PIN_8
#define IIC_Pin_SDA		GPIO_PIN_9
#define SCL_H         HAL_GPIO_WritePin(GPIOA,IIC_Pin_SCL,GPIO_PIN_SET)
#define SCL_L         HAL_GPIO_WritePin(GPIOA,IIC_Pin_SCL,GPIO_PIN_RESET)
#define SDA_H         HAL_GPIO_WritePin(GPIOA,IIC_Pin_SDA,GPIO_PIN_SET)
#define SDA_L         HAL_GPIO_WritePin(GPIOA,IIC_Pin_SDA,GPIO_PIN_RESET)
#define SCL_read      HAL_GPIO_ReadPin(GPIOA,IIC_Pin_SCL)
#define SDA_read      HAL_GPIO_ReadPin(GPIOA,IIC_Pin_SDA)

//IIC所有操作函数
void SDA_IN(void);
void SDA_OUT(void);
void IIC_Configuration(void);                //初始化IIC的IO口			

unsigned char IIC_Start(void);
void IIC_Stop(void);

void IIC_Ack(void);
void IIC_NoAck(void);
unsigned char IIC_WaitAck(void); 	 //返回为:=1有ACK,=0无ACK

void IIC_SendByte(unsigned char SendByte); //数据从高位到低位//
u8 IIC_ReadByte(unsigned char ack);

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
#endif

