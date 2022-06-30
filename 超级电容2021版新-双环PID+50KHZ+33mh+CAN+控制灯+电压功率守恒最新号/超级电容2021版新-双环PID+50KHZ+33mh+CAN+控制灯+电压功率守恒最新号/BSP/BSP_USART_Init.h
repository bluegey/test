#ifndef _BSP_USART_INIT_H_
#define _BSP_USART_INIT_H_
#include "stm32f10x.h"                  // Device header



void BSP_USART2_Init(u32 BaudRate);
void Send_Data_Char(u8 data);
void USART_Send_BUFF(u8 *buff,u8 len);
void DMA_init(void);
#define USART2_Channel_Num 100
#define FRAME_HEADER   0xA5   //֡ͷ

extern u8 USART2_Get_Channel[USART2_Channel_Num];
#endif

