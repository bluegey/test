#ifndef __IIC_H__
#define __IIC_H__
#include "sys.h"
/*
**************************************************************************************************
宏及变量定义
**************************************************************************************************
*/
#define SCL_H 		PBout(10) = 1		//IIC时钟输出高
#define SCL_L 		PBout(10) = 0	  	//IIC时钟输出低

#define SDA_Out_H 	GPIOB -> CRH & (~(0x0000000F << 12)) | (0x00000003 << 12); PBout(11) = 1		//IIC数据输出高
#define SDA_Out_L 	GPIOB -> CRH & (~(0x0000000F << 12)) | (0x00000003 << 12); PBout(11) = 0	 	//IIC数据输出低
#define SDA_In		GPIOB -> IDR & GPIO_Pin_11		//IIC读数据
/*
****************************************************************************************************
函数定义
****************************************************************************************************
*/
void IIC_GPIO_Init(void);		//IIC端口初始化
void IIC_Delay(void);
void IIC_Delay5ms(void);
bool IIC_Start(void);		 	//IIC传输开始
void IIC_Stop(void);			//IIC传输结束
void IIC_Ack(void);				//IIC应答
void IIC_NoAck(void);			//IIC无应答
bool IIC_WaitAck(void);
void IIC_SendByte(u8 SendByte);	//发送字节
u8 IIC_ReadByte(void);			//读取字节
bool IIC_SingleWrite(u8 SLAVE_Address, u8 REG_Address, u8 REG_Data);		//单字节写
u8 IIC_SingleRead(u8 SLAVE_Address, u8 REG_Address);

#endif
