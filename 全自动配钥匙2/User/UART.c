#include "UART.h"
#include "stdio.h"
#include "stdarg.h"


/*********************************************************************
*��������:	USART1_Init
*��������:	��ʼ������ģ��
*�������:		void
*�������:		void
*��ע:		
*********************************************************************/

void UART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate=115200;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART_InitStructure);
	USART_Cmd(USART1,ENABLE);
}



/*********************************************************************
*��������:	UART_Send_Byte
*��������:	����һ�ֽ�����
*�������:	ch--Ҫ���͵�һ���ֽ�
*�������:	void
*��ע:		
*********************************************************************/

void UART_Send_Byte(unsigned char ch)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
	USART_SendData(USART1,ch);
}


/********************************************************************* 
�������ƣ�UART_Send_Enter
�������ܣ��س�����
��ڲ����� 
���ڲ����� 
�� ע�� ��Ƭ������0d 0a�������ֽڣ��ڡ������նˡ��ϻ��лس����е�Ч�� 
*********************************************************************/

void UART_Send_Enter(void)
{
  UART_Send_Byte(0x0d);
  UART_Send_Byte(0x0a);
}



/*********************************************************************
*��������:	UART_Send_Strs
*��������:	
*�������:	�ַ�ָ��
*�������:	void
*��ע:		
*********************************************************************/

void UART_Send_Strs(char *ptr)
{
  while(*ptr!='\0')
  {
    USART_SendData(USART1,*ptr++);
    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
  }
}
	

/*********************************************************************
*��������:	UART_Printf
*��������:	��ʽ�����
*�������:	
*�������:	void
*��ע:		д���ַ���Ҫ����64���ַ�
*********************************************************************/
   
void UART_Printf(const char *fmt,...)  
{
	va_list ap;
	char string[64]={0};
	va_start(ap,fmt);
	vsnprintf(string,64,fmt,ap);
	va_end(ap);
	UART_Send_Strs(string);
}





