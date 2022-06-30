#ifndef _ADC_TASK_H_
#define _ADC_TASK_H_

#include "init.h"


typedef struct 
{
   float Ch1;
	 float Ch2;
	 float Ch3;
	 float Ch4;
	 float Ch_Ref;
	 float Ch_In;//通道之和
	 float ch_err; //通道与裁判系统误差
}Current;

#define Offest_flag 0x05
#define Count 50   //采集50次后开始计算

extern Current Real_Current;//实际流经的电流
extern Current Raw_Current;//实际流经的电流
extern Current Offest_Current;  //基准值

void ADC1_Data_processing(void);
#endif



