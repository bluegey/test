#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//LED�˿ڶ���
#define LED0 PFout(9)	// DS0
#define LED1 PFout(10)	// DS1	 

void LED_Init(void);//��ʼ��
#ifdef __cplusplus
	}
#endif
	
#endif
