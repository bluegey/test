#include "iic.h"
#include "delay.h"
#include "gpio.h"
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

//IIC��ʼ��
void IIC_Configuration(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();   //ʹ��GPIOAʱ��
    
    //PH4,5��ʼ������
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
  __HAL_RCC_GPIOA_CLK_ENABLE();   //ʹ��GPIOBʱ��
	
  GPIO_InitStructure.Pin = IIC_Pin_SDA;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull=GPIO_NOPULL;        //
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
}
void SDA_OUT()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
 __HAL_RCC_GPIOA_CLK_ENABLE();   //ʹ��GPIOBʱ��
	
  GPIO_InitStructure.Pin = IIC_Pin_SDA;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull=GPIO_NOPULL;          	//
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
}

//����IIC��ʼ�ź�
unsigned char IIC_Start(void)
{
	SDA_OUT();     //sda�����
	SDA_H;
	SCL_H;
	delay_us(4);
 	SDA_L;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	SCL_L;//ǯסI2C���ߣ�׼�����ͻ�������� 
	return 0;
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	SCL_L;
	SDA_L;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	SCL_H;
	SDA_H;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
unsigned char IIC_WaitAck(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
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
	SCL_L;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_SendByte(u8 txd)
{                        
  u8 t;   
	  SDA_OUT(); 	    
    SCL_L;//����ʱ�ӿ�ʼ���ݴ���
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_ReadByte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
        IIC_NoAck();//����nACK
    else
        IIC_Ack(); //����ACK
    return receive;
}


