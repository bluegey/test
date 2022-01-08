#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"


//#define KEY0 PEin(4)   	//PE4
//#define KEY1 PEin(3)	//PE3 
//#define WK_UP PAin(0)	//PA0  WK_UP

#define KEY7       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)//       上
#define KEY4       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)//       下
#define KEY6       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)//       左
#define KEY0       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//       右
#define KEY5       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)//       中（确认）
#define KEY1       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)//       返回
 

#define KEY_UP 	      1	//   上
#define KEY_DOWN	    2	//   下
#define KEY_LEFT      3	//   左
#define KEY_RIGHT 	  4	//   右
#define KEY_CONFRIM 	5	//   确认
#define KEY_RETURN 	  6	//   返回

void KEY_Init(void);//IO初始化
u8 KEY_Scan(u8);  	//按键扫描函数					    
#endif
