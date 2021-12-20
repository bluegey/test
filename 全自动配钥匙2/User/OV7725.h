#ifndef _OV7725_H_
#define _OV7725_H_

#include "stm32f10x.h"


typedef struct
{
	uint32_t self;
	uint32_t prior;				//ǰһ��ͼ���ַ
	uint32_t next;				//��һ��ͼ���ַ
	uint16_t serial_num;	//ͼ����
	uint8_t  type;
	uint8_t  reserve;
}Image_Info_TypeDef;		//����ͼ��ṹ��


typedef struct
{
	uint32_t first;				//��һ��ͼ���ַ
	uint32_t last;				//���һ��ͼ���ַ
	uint16_t total;				//ͼ������
}Storage_Info_TypeDef;	//���ݴ洢�ṹ��



uint8_t OV7725_init(void);


#endif
