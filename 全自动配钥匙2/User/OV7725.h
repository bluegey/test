#ifndef _OV7725_H_
#define _OV7725_H_

#include "stm32f10x.h"


typedef struct
{
	uint32_t self;
	uint32_t prior;				//前一幅图像地址
	uint32_t next;				//后一幅图像地址
	uint16_t serial_num;	//图像编号
	uint8_t  type;
	uint8_t  reserve;
}Image_Info_TypeDef;		//单幅图像结构体


typedef struct
{
	uint32_t first;				//第一幅图像地址
	uint32_t last;				//最后一幅图像地址
	uint16_t total;				//图像总数
}Storage_Info_TypeDef;	//数据存储结构体



uint8_t OV7725_init(void);


#endif
