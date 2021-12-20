#ifndef	_UART_H_
#define	_UART_H_

#include "stm32f10x.h"


void UART_Init(void);
void UART_Printf(const char *fmt,...); 
void UART_Send_Byte(unsigned char ch);
void UART_Send_Strs(char *ptr);
void UART_Send_Enter(void);

#endif
