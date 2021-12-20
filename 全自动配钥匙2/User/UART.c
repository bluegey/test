#include "UART.h"
#include "stdio.h"
#include "stdarg.h"


/*********************************************************************
*函数名称:	USART1_Init
*函数功能:	初始化串口模块
*输入参数:		void
*输出参数:		void
*备注:		
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
*函数名称:	UART_Send_Byte
*函数功能:	发送一字节数据
*输入参数:	ch--要发送的一个字节
*输出参数:	void
*备注:		
*********************************************************************/

void UART_Send_Byte(unsigned char ch)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
	USART_SendData(USART1,ch);
}


/********************************************************************* 
函数名称：UART_Send_Enter
函数功能：回车换行
入口参数： 
出口参数： 
备 注： 单片机发送0d 0a这两个字节，在“超级终端”上会有回车换行的效果 
*********************************************************************/

void UART_Send_Enter(void)
{
  UART_Send_Byte(0x0d);
  UART_Send_Byte(0x0a);
}



/*********************************************************************
*函数名称:	UART_Send_Strs
*函数功能:	
*输入参数:	字符指针
*输出参数:	void
*备注:		
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
*函数名称:	UART_Printf
*函数功能:	格式化输出
*输入参数:	
*输出参数:	void
*备注:		写入字符不要超过64个字符
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





