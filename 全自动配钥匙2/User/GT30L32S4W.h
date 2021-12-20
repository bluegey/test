#ifndef	 _GT30L32S4W_H_
#define	 _GT30L32S4W_H_

#include "stm32f10x.h"


/**********************×Ö·ûÀàÐÍ*********************/
#define	Chinese_1112			1
#define	Chinese_1516			2
#define	Chinese_2424			3
#define	Chinese_3232			4

#define	ASCII_0507				5
#define	ASCII_0708				6
#define	ASCII_0612				7
#define	ASCII_0816				8
#define	ASCII_1224				9
#define	ASCII_1632				10

#define	ExChar_0612				11
#define	ExChar_0816				12
#define	ExChar_1224				13
#define	ExChar_1632				14


void SPI_FLASH_Init(void);
uint32_t	GetChineseAddress(uint8_t type,uint8_t* GBCode,uint8_t* nbyte,uint8_t* ncols,uint8_t* nrows);
void GT30L32S4W_Read(uint32_t address,uint8_t nbyte,uint8_t* pBuffer);


#endif
