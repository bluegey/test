#ifndef __DEVICER_USART_H
#define __DEVICER_USART_H

#include "init.h"




#define  BSP_UART4_DMA_RX_BUF_LEN               30u



typedef struct
{
    u16 len;
    uint8_t buffer[50];
} datarevice;

void USART1_Init(u32 My_BaudRate);



#endif
