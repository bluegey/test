#include "EEPROM.h" 
#include "mydelay.h"




#define SCL_H()        	GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define SCL_L()        	GPIO_ResetBits(GPIOB,GPIO_Pin_6)
   
#define SDA_H()        	GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define SDA_L()        	GPIO_ResetBits(GPIOB,GPIO_Pin_7)

#define SDA_read()    	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)

unsigned char I2C_Start(void)
{
	SDA_H();
	SCL_H();
	my_delayus(5);
	if(!SDA_read())
		return 0;	//SDA线为低电平则总线忙,退出
	SDA_L();
	my_delayus(5);
	if(SDA_read()) 
		return 0;	//SDA线为高电平则总线出错,退出
	SDA_L();
	my_delayus(5);
	return 1;
}

void I2C_Stop(void)
{
	SCL_L();
	my_delayus(5);
	SDA_L();
	my_delayus(5);
	SCL_H();
	my_delayus(5);
	SDA_H();
	my_delayus(5);
} 

void I2C_Ack(void)
{	
	SCL_L();
	my_delayus(5);
	SDA_L();
	my_delayus(5);
	SCL_H();
	my_delayus(5);
	SCL_L();
	my_delayus(5);
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{	
	SCL_L();
	my_delayus(5);
	SDA_H();
	my_delayus(5);
	SCL_H();
	my_delayus(5);
	SCL_L();
	my_delayus(5);
} 

unsigned char I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	SCL_L();
	my_delayus(5);
	SDA_H();			
	my_delayus(5);
	SCL_H();
	my_delayus(5);
	if(SDA_read())
	{
		SCL_L();
		my_delayus(5);
		return 0;
	}
	SCL_L();
	my_delayus(5);
	return 1;
}

void I2C_SendByte(unsigned char SendByte) //数据从高位到低位//
{
	unsigned char i=8;
	while(i--)
	{
		SCL_L();
		if(SendByte&0x80)	//	 10000000	如果最高位位一则返回
			SDA_H();  
		else 
			SDA_L();   
		SendByte<<=1;//sendbyte= sendbyte<<1 一位一位的发送
		my_delayus(5);
		SCL_H();
		my_delayus(5);
	}
	SCL_L();
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //数据从高位到低位//
{ 
	unsigned char i=8;
	unsigned char ReceiveByte=0;

	SDA_H();				
	while(i--)
	{
		ReceiveByte<<=1;      
		SCL_L();
		my_delayus(5);
		SCL_H();
		my_delayus(5);	
		if(SDA_read())
		{
			ReceiveByte|=0x01;
		}
	}
	SCL_L();
	return ReceiveByte;
} 
        


//初始化IIC接口
void AT24CXX_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	AT24CXX_Check();
}
//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr)
{				  
	uint8_t temp=0;		  	    																 
  I2C_Start();  
	I2C_SendByte(0XA0+((ReadAddr/256)<<1));   //发送器件地址0XA0,写数据 	 
	I2C_WaitAck(); 
  I2C_SendByte(ReadAddr%256);   //发送低地址
	I2C_WaitAck();	    
	I2C_Start();  	 	   
	I2C_SendByte(0XA1);           //进入接收模式			   
	I2C_WaitAck();	 
	temp=I2C_RadeByte();		   
	I2C_Stop();//产生一个停止条件	    
	return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite)
{				   	  	    																 
  I2C_Start();  
	I2C_SendByte(0XA0+((WriteAddr/256)<<1));   //发送器件地址0XA0,写数据 
	I2C_WaitAck();	   
  I2C_SendByte(WriteAddr%256);   //发送低地址
	I2C_WaitAck(); 	 										  		   
	I2C_SendByte(DataToWrite);     //发送字节							   
	I2C_WaitAck();  		    	   
  I2C_Stop();//产生一个停止条件
	my_delayms(5);
}


//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
uint8_t AT24CXX_Check(void)
{
	uint8_t temp;
	temp=AT24CXX_ReadOneByte(255);//避免每次开机都写AT24CXX			   
	if(temp==0X55)
		return 0;		   
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(255,0X55);
	  temp=AT24CXX_ReadOneByte(255);	  
		if(temp==0X55)
			return 0;
	}
	return 1;											  
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	uint8_t i;
	for(i=0;i<NumToRead;i++)
	{
		*pBuffer=AT24CXX_ReadOneByte(ReadAddr);
		ReadAddr++;
		pBuffer++;
	}
}  
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
	uint8_t i;
	for(i=0;i<NumToWrite;i++)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}
 







