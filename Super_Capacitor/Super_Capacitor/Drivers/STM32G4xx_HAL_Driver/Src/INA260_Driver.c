#include "iic.h"
#include "INA260_Driver.h"
#define N 12


void INA_REG_Write(u8 reg,u16 data)
{
	u8 data_temp[2];
	data_temp[0]=(u8)(data>>8);
	data_temp[1]=(u8)(data & 0xff);
	
	IIC_Start();
	IIC_SendByte(INA260_ADDRESS);
	IIC_WaitAck();
	IIC_SendByte(reg);
	IIC_WaitAck();
	IIC_SendByte(data_temp[0]);
	IIC_WaitAck();
	data++;
	IIC_SendByte(data_temp[1]);
	IIC_WaitAck();
	IIC_Stop();
}

void INA_Read_Byte(u8 reg,u8 *data)
{
	IIC_Start();
	IIC_SendByte(INA260_ADDRESS);
	IIC_WaitAck();
	IIC_SendByte(reg);
	IIC_WaitAck();
	
	IIC_Start();
	IIC_SendByte(INA260_ADDRESS|0x01);
	IIC_WaitAck();
	
	*data=IIC_ReadByte(1);
	data++;
	*data=IIC_ReadByte(0);
	IIC_Stop();
}

void INA_Init()
{
	IIC_Configuration();
	INA_REG_Write(0x00,0x6527);
}

int INA_Get_Current_mA()
{
	u8 data_temp[2];
	int current=0;
	INA_Read_Byte(0x01,data_temp);
	if(data_temp[0]>>7)//最高位是负值的标志位，为1则进行补码转换
	{
		current=((!(((data_temp[0]<<8)|data_temp[1])-1))*-1.25);
	}
	else
	{
		current=(int)(((data_temp[0]<<8)|data_temp[1])*1.25);
	}
	return current;
}

u16 INA_Get_Voltage_mV()
{
	u8 data_temp[2];
	INA_Read_Byte(0x02,data_temp);
	return (int)(((data_temp[0]<<8)|data_temp[1])*1.25);
}

u16 INA_Get_Power_mW()
{
	u8 data_temp[2];
	INA_Read_Byte(0x03,data_temp);
	return (int)(((data_temp[0]<<8)|data_temp[1]));
}

u8 INA260_DataUpdate()
{
	INAReal_Data.voltage = INA_Get_Voltage_mV();
	INAReal_Data.current = INA_Get_Current_mA();
	INAReal_Data.power   = INA_Get_Power_mW();
//	if(INAReal_Data.voltage==0x0000||INAReal_Data.voltage==0xffff)
//	{
//		
//	}
//	else
//	{
////	Real_Monitor.INA260FrameCounter++;
//		return 1;
//	}	
	return 0;
}
//---------------------------
char INA260_Filter()
{
char count;

char value_buf[N];
int k = 0;
for (count=0;count<N;count++)

{

value_buf[count] = INA_Get_Current_mA();

//delay();

}


for(int i = 1; i < N; i++) 
{
  if(value_buf[i] > value_buf[k]) k = i;
} 
//array[k] 就是最大值
//k 就是对应下标

return value_buf[k];
}


