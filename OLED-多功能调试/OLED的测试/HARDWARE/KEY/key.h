#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"


//#define KEY0 PEin(4)   	//PE4
//#define KEY1 PEin(3)	//PE3 
//#define WK_UP PAin(0)	//PA0  WK_UP

#define KEY7       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)//       ��
#define KEY4       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)//       ��
#define KEY6       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)//       ��
#define KEY0       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//       ��
#define KEY5       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)//       �У�ȷ�ϣ�
#define KEY1       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)//       ����
 

#define KEY_UP 	      1	//   ��
#define KEY_DOWN	    2	//   ��
#define KEY_LEFT      3	//   ��
#define KEY_RIGHT 	  4	//   ��
#define KEY_CONFRIM 	5	//   ȷ��
#define KEY_RETURN 	  6	//   ����

void KEY_Init(void);//IO��ʼ��
u8 KEY_Scan(u8);  	//����ɨ�躯��					    
#endif
