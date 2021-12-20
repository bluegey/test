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
		return 0;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	SDA_L();
	my_delayus(5);
	if(SDA_read()) 
		return 0;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
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

unsigned char I2C_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
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

void I2C_SendByte(unsigned char SendByte) //���ݴӸ�λ����λ//
{
	unsigned char i=8;
	while(i--)
	{
		SCL_L();
		if(SendByte&0x80)	//	 10000000	������λλһ�򷵻�
			SDA_H();  
		else 
			SDA_L();   
		SendByte<<=1;//sendbyte= sendbyte<<1 һλһλ�ķ���
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
unsigned char I2C_RadeByte(void)  //���ݴӸ�λ����λ//
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
        


//��ʼ��IIC�ӿ�
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
//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr)
{				  
	uint8_t temp=0;		  	    																 
  I2C_Start();  
	I2C_SendByte(0XA0+((ReadAddr/256)<<1));   //����������ַ0XA0,д���� 	 
	I2C_WaitAck(); 
  I2C_SendByte(ReadAddr%256);   //���͵͵�ַ
	I2C_WaitAck();	    
	I2C_Start();  	 	   
	I2C_SendByte(0XA1);           //�������ģʽ			   
	I2C_WaitAck();	 
	temp=I2C_RadeByte();		   
	I2C_Stop();//����һ��ֹͣ����	    
	return temp;
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite)
{				   	  	    																 
  I2C_Start();  
	I2C_SendByte(0XA0+((WriteAddr/256)<<1));   //����������ַ0XA0,д���� 
	I2C_WaitAck();	   
  I2C_SendByte(WriteAddr%256);   //���͵͵�ַ
	I2C_WaitAck(); 	 										  		   
	I2C_SendByte(DataToWrite);     //�����ֽ�							   
	I2C_WaitAck();  		    	   
  I2C_Stop();//����һ��ֹͣ����
	my_delayms(5);
}


//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
uint8_t AT24CXX_Check(void)
{
	uint8_t temp;
	temp=AT24CXX_ReadOneByte(255);//����ÿ�ο�����дAT24CXX			   
	if(temp==0X55)
		return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(255,0X55);
	  temp=AT24CXX_ReadOneByte(255);	  
		if(temp==0X55)
			return 0;
	}
	return 1;											  
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
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
//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
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
 







