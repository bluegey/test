#ifndef __BSP_IIC_INIT_H_
#define __BSP_IIC_INIT_H_

#include "stm32f10x.h"

void IIC_Configuration(void);
void SDA_IN(void);
void SDA_OUT(void);
void IIC_delay(u16 time);

unsigned char IIC_Start(void);
void IIC_Stop(void);

void IIC_Ack(void);
void IIC_NoAck(void);
unsigned char IIC_WaitAck(void); 	 //返回为:=1有ACK,=0无ACK

void IIC_SendByte(unsigned char SendByte); //数据从高位到低位//
u8 IIC_ReadByte(unsigned char ack);



#endif















