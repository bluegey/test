#ifndef __IIC_H__
#define __IIC_H__
#include "sys.h"
/*
**************************************************************************************************
�꼰��������
**************************************************************************************************
*/
#define SCL_H 		PBout(10) = 1		//IICʱ�������
#define SCL_L 		PBout(10) = 0	  	//IICʱ�������

#define SDA_Out_H 	GPIOB -> CRH & (~(0x0000000F << 12)) | (0x00000003 << 12); PBout(11) = 1		//IIC���������
#define SDA_Out_L 	GPIOB -> CRH & (~(0x0000000F << 12)) | (0x00000003 << 12); PBout(11) = 0	 	//IIC���������
#define SDA_In		GPIOB -> IDR & GPIO_Pin_11		//IIC������
/*
****************************************************************************************************
��������
****************************************************************************************************
*/
void IIC_GPIO_Init(void);		//IIC�˿ڳ�ʼ��
void IIC_Delay(void);
void IIC_Delay5ms(void);
bool IIC_Start(void);		 	//IIC���俪ʼ
void IIC_Stop(void);			//IIC�������
void IIC_Ack(void);				//IICӦ��
void IIC_NoAck(void);			//IIC��Ӧ��
bool IIC_WaitAck(void);
void IIC_SendByte(u8 SendByte);	//�����ֽ�
u8 IIC_ReadByte(void);			//��ȡ�ֽ�
bool IIC_SingleWrite(u8 SLAVE_Address, u8 REG_Address, u8 REG_Data);		//���ֽ�д
u8 IIC_SingleRead(u8 SLAVE_Address, u8 REG_Address);

#endif
