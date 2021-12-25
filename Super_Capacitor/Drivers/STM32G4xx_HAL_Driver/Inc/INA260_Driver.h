#ifndef __INA260_DRIVER_H__
#define __INA260_DRIVER_H__

#include "main.h"

#define INA260_ADDRESS (0x40<<1)

typedef struct 
{
  u16 voltage;
	int current;
	u16 power;
}InaReal_Data;

void INA_REG_Write(u8 reg,u16 data);
void INA_Read_Byte(u8 reg,u8 *data);
void INA_Init(void);
char INA260_Filter(void);
u16 INA_Get_Voltage_mV(void);
u16 INA_Get_Power_mW(void);
int INA_Get_Current_mA(void);
u8 INA260_DataUpdate(void);
extern InaReal_Data  INAReal_Data;
#endif
