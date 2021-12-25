#ifndef _MYIIC_H
#define _MYIIC_H
#include "main.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F103������
//IIC��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2019/9/18
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
//IO��������
//#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}	//PB7����ģʽ
//#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;} 	//PB7���ģʽ

//IO����
#define IIC_Pin_SCL		GPIO_PIN_8
#define IIC_Pin_SDA		GPIO_PIN_9
#define SCL_H         HAL_GPIO_WritePin(GPIOA,IIC_Pin_SCL,GPIO_PIN_SET)
#define SCL_L         HAL_GPIO_WritePin(GPIOA,IIC_Pin_SCL,GPIO_PIN_RESET)
#define SDA_H         HAL_GPIO_WritePin(GPIOA,IIC_Pin_SDA,GPIO_PIN_SET)
#define SDA_L         HAL_GPIO_WritePin(GPIOA,IIC_Pin_SDA,GPIO_PIN_RESET)
#define SCL_read      HAL_GPIO_ReadPin(GPIOA,IIC_Pin_SCL)
#define SDA_read      HAL_GPIO_ReadPin(GPIOA,IIC_Pin_SDA)

//IIC���в�������
void SDA_IN(void);
void SDA_OUT(void);
void IIC_Configuration(void);                //��ʼ��IIC��IO��			

unsigned char IIC_Start(void);
void IIC_Stop(void);

void IIC_Ack(void);
void IIC_NoAck(void);
unsigned char IIC_WaitAck(void); 	 //����Ϊ:=1��ACK,=0��ACK

void IIC_SendByte(unsigned char SendByte); //���ݴӸ�λ����λ//
u8 IIC_ReadByte(unsigned char ack);

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
#endif

