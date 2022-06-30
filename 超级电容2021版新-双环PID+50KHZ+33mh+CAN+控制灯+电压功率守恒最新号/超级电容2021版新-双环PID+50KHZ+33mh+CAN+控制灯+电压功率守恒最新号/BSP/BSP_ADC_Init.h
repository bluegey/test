#ifndef _BSP_ADC_INIT_H_
#define _BSP_ADC_INIT_H_

#include "stm32f10x.h"                  // Device header
//需要转化的ADC通道数
#define ADC1_Channel_Num 5   
void BSP_ADC1_Init(void);
static void DMA_forADC1_Init(void);
extern 	float adc_voltage;
extern 	float Real_Voltage;
//extern u16 ADC1_Real_Channel[ADC1_Channel_Num];
u16 Get_Adc(u8 ch);
u16 Get_Adc_Average(u8 ch,u8 times);
#endif


