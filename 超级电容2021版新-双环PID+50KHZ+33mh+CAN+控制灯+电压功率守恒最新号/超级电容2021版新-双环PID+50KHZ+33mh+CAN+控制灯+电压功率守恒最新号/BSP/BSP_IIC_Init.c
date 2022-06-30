#include "Init.h"


#define IIC_Pin_SCL		GPIO_Pin_8
#define IIC_Pin_SDA		GPIO_Pin_9
#define SCL_H         GPIO_SetBits(GPIOA,IIC_Pin_SCL)
#define SCL_L         GPIO_ResetBits(GPIOA,IIC_Pin_SCL)
#define SDA_H         GPIO_SetBits(GPIOA,IIC_Pin_SDA)
#define SDA_L         GPIO_ResetBits(GPIOA,IIC_Pin_SDA)
#define SCL_read      GPIO_ReadInputDataBit(GPIOA,IIC_Pin_SCL)
#define SDA_read      GPIO_ReadInputDataBit(GPIOA,IIC_Pin_SDA)


void IIC_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  IIC_Pin_SCL | IIC_Pin_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	SCL_H;
	SDA_H;
}

void SDA_IN()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = IIC_Pin_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
}
void SDA_OUT()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = IIC_Pin_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
}

void IIC_delay(u16 time)
{
   u16 i=0;
   while(time--)
   {
      i=10;//自己定义
      while(i--);
   }
}
///////////////////////////////////////////////////////////////
unsigned char IIC_Start(void)
{
	SDA_OUT();     //sda线输出

	SDA_H;
	SCL_H;
	IIC_delay(4);
	SDA_L;
	IIC_delay(4);
	SCL_L;
	return 1;
}

void IIC_Stop(void)
{
	SDA_OUT();//sda线输出

	SCL_L;
	SDA_L;
	IIC_delay(4);
	SCL_H;
	SDA_H;
	IIC_delay(4);
}
//////////////////////////////////////////////////////////////
void IIC_Ack(void)
{	
	SCL_L;
	SDA_OUT();
	SDA_L;
	IIC_delay(2);
	SCL_H;
	IIC_delay(2);
	SCL_L;
}

void IIC_NoAck(void)
{
	SCL_L;
	SDA_OUT();
	SDA_H;
	IIC_delay(2);
	SCL_H;
	IIC_delay(2);
	SCL_L;
}

unsigned char IIC_WaitAck(void)
{
	u8 ucErrTime=0;

	SDA_IN();      //SDA设置为输入
	SDA_H;
	IIC_delay(1);
	SCL_H;
	IIC_delay(1);
	while(SDA_read)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_L;  
	return 0;  
}
////////////////////////////////////////////////////////////////
void IIC_SendByte(u8 txd)
{                        
    u8 t;   
	  SDA_OUT(); 	    
    SCL_L;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
			if((txd&0x80)>>7)
			{
				SDA_H;
			}
			else
			{
				SDA_L;
			}
			txd<<=1;
			IIC_delay(2);
				SCL_H;
			IIC_delay(2);
				SCL_L;
			IIC_delay(2);
    }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 IIC_ReadByte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
    SCL_L;
    IIC_delay(2);
		SCL_H;
    receive<<=1;
    if(SDA_read)receive++;
		IIC_delay(1);
  }
    if (!ack)
        IIC_NoAck();//发送nACK
    else
        IIC_Ack(); //发送ACK
    return receive;
}
////////////////////////////////////////////////////////////////










