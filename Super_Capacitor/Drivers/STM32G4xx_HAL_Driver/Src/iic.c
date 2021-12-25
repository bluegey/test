#include "iic.h"
#include "delay.h"
#include "gpio.h"
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

//IIC初始化
void IIC_Configuration(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();   //使能GPIOA时钟
    
    //PH4,5初始化设置
    GPIO_Initure.Pin=IIC_Pin_SCL|IIC_Pin_SDA;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_NOPULL;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);
    
    SCL_L;
    SDA_L;  
}
void SDA_IN()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  __HAL_RCC_GPIOA_CLK_ENABLE();   //使能GPIOB时钟
	
  GPIO_InitStructure.Pin = IIC_Pin_SDA;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull=GPIO_NOPULL;        //
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
}
void SDA_OUT()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
 __HAL_RCC_GPIOA_CLK_ENABLE();   //使能GPIOB时钟
	
  GPIO_InitStructure.Pin = IIC_Pin_SDA;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull=GPIO_NOPULL;          	//
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
}

//产生IIC起始信号
unsigned char IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	SDA_H;
	SCL_H;
	delay_us(4);
 	SDA_L;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	SCL_L;//钳住I2C总线，准备发送或接收数据 
	return 0;
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	SCL_L;
	SDA_L;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	SCL_H;
	SDA_H;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
unsigned char IIC_WaitAck(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	SDA_H;
	delay_us(1);	   
	SCL_H;
	delay_us(1);	 
	while(SDA_read)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_L;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	SCL_L;
	SDA_OUT();
	SDA_L;
	delay_us(2);
	SCL_H;
	delay_us(2);
	SCL_L;
}
//不产生ACK应答		    
void IIC_NoAck(void)
{
	SCL_L;
	SDA_OUT();
	SDA_H;
	delay_us(2);
	SCL_H;
	delay_us(2);
	SCL_L;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
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
			delay_us(2);
				SCL_H;
			delay_us(2);
				SCL_L;
			delay_us(2);
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
    delay_us(2);
		SCL_H;
    receive<<=1;
    if(SDA_read)receive++;
		delay_us(1);
  }
    if (!ack)
        IIC_NoAck();//发送nACK
    else
        IIC_Ack(); //发送ACK
    return receive;
}


