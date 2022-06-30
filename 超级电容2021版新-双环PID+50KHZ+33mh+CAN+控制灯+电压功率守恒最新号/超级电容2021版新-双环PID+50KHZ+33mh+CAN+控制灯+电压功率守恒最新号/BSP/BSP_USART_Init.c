#include "init.h"
u8 USART2_Get_Channel[USART2_Channel_Num];

void BSP_USART2_Init(u32 BaudRate)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure; 
	USART_ClockInitTypeDef USART_ClockInitStruct;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  //配置USART2时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
	USART_ClockInit(USART2, &USART_ClockInitStruct);

  USART_Init(USART2, &USART_InitStructure); //初始化串口2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接收中断

  USART_Cmd(USART2, ENABLE);                //使能串口2
}

void Send_Data_Char(u8 data)
{		
	while((USART2->SR&0X40) == 0);
	USART_SendData(USART2,data);
}	
void USART_Send_BUFF(u8 *buff,u8 len)
{
	u8 i,temp;
	for(i=0;i<len;i++)
	{
		temp=*buff++;
		Send_Data_Char(temp);
	}
}
extern osMessageQueueId_t UARTRX_MessageQueue;
void USART2_IRQHandler(void)                	//串口中断服务程序
{
	u8 Res;

	if(USART2->SR & USART_SR_ORE)//ORE溢出中断
	{
		u8 com_data = USART2->DR;
	}
	
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!= RESET)//串口接收中断
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	  Res =USART_ReceiveData(USART2);	//读取接收到的数据		
		osMessageQueuePut(UARTRX_MessageQueue,&Res,osPriorityLow1,0);//消息队列发送数据
		
	 }
	
}
//#include "init.h"
//u8 USART2_Get_Channel[USART2_Channel_Num];
//u8 receive_data[USART2_Channel_Num];	
//u16 USART2_RX_STA=0;       //接收状态标记
//void BSP_USART2_Init(u32 BaudRate)
//{
//  //GPIO端口设置
//  GPIO_InitTypeDef GPIO_InitStructure; 
//	USART_ClockInitTypeDef USART_ClockInitStruct;
//	USART_InitTypeDef USART_InitStructure;
//	DMA_InitTypeDef    DMA_Initstructure;  
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE); 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//	USART_InitStructure.USART_BaudRate = BaudRate;//串口波特率
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

//  //配置USART2时钟
//	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
//	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
//	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
//	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
//	USART_ClockInit(USART2, &USART_ClockInitStruct);

//  USART_Init(USART2, &USART_InitStructure); //初始化串口2
//	
//	 USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);//开启DMA接收
//		DMA_Initstructure.DMA_PeripheralBaseAddr =  (u32)(&USART2->DR); 
//DMA_Initstructure.DMA_MemoryBaseAddr     = (u32)USART2_Get_Channel;  
//DMA_Initstructure.DMA_DIR = DMA_DIR_PeripheralSRC;   
//DMA_Initstructure.DMA_BufferSize = 128;   
//DMA_Initstructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   
//DMA_Initstructure.DMA_MemoryInc =DMA_MemoryInc_Enable;  
//DMA_Initstructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   
//DMA_Initstructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
//DMA_Initstructure.DMA_Mode = DMA_Mode_Normal;  
//DMA_Initstructure.DMA_Priority = DMA_Priority_High;  
//DMA_Initstructure.DMA_M2M = DMA_M2M_Disable;   
//DMA_Init(DMA1_Channel6,&DMA_Initstructure);     //启动DMA   
//DMA_Cmd(DMA1_Channel6,ENABLE); 
//		
////  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接收中断
//	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//开启空闲中断

//  USART_Cmd(USART2, ENABLE);                //使能串口2
//	
//// DMA_init();
//}

//void Send_Data_Char(u8 data)
//{		
//	while((USART2->SR&0X40) == 0);
//	USART_SendData(USART2,data);
//}	
//void USART_Send_BUFF(u8 *buff,u8 len)
//{
//	u8 i,temp;
//	for(i=0;i<len;i++)
//	{
//		temp=*buff++;
//		Send_Data_Char(temp);
//	}
//}

//void USART2_IRQHandler(void)                	//串口中断服务程序
//{
// static uint32_t this_time_rx_len = 0;
//	unsigned char num=0;   
//	static u16 length;u8 Res;
//if(USART_GetITStatus(USART2,USART_IT_IDLE) == SET)    
//{    
////		  Res =USART_ReceiveData(USART2);	//读取接收到的数据	 
//num = USART2->SR;      
//num = USART2->DR; //清USART_IT_IDLE标志       
//DMA_Cmd(DMA1_Channel6,DISABLE);    //关闭DMA       
//this_time_rx_len = 50 - DMA_GetCurrDataCounter(DMA1_Channel6);  
//	
//	if(this_time_rx_len==judgement_length){
//	    if((USART2_RX_STA&0x8000)==0)//接收未完成
//		{
//			if((USART2_RX_STA&0x4000)==0)//没找到帧头
//			{
//				if(Res==FRAME_HEADER)//数据帧头
//				{
//					USART2_RX_STA|=0x4000;
//					USART2_Get_Channel[USART2_RX_STA&0x3FFF]=Res;//将帧头存入数据
//					USART2_RX_STA++;
//				}
//			}
//			else//找到帧头
//			{
//					USART2_Get_Channel[USART2_RX_STA&0x3FFF]=Res;
//					USART2_RX_STA++;
//					if((USART2_RX_STA & 0x3fff) == 3)//接收到长度信息
//					{
//						length = USART2_Get_Channel[1] | ((u16)Res << 8); 
//					}
//					if((USART2_RX_STA & 0x3fff) == length + 5 + 2 + 2)
//					{
//						USART2_RX_STA|=0x8000;//接收完成了  
//					
//					}
//					if((USART2_RX_STA&0X3FFF)>(USART2_Channel_Num-1))USART2_RX_STA=0;//接收数据错误,重新开始接收									
//			}
//		}
//	}

//num = 128 -  DMA_GetCurrDataCounter(DMA1_Channel6);      //得到真正接收数据个数        
//receive_data[num] = '\0';       DMA1_Channel6->CNDTR=128;       //重新设置接收数据个数          
//DMA_Cmd(DMA1_Channel6,ENABLE);  //开启DMA     
////  receive_flag = 1;           //接收数据标志位置1   
//}
//	
//	
//	

////	if(USART2->SR & USART_SR_ORE)//ORE溢出中断
////	{
////		u8 com_data = USART2->DR;
////	}
////	
////	if(USART_GetITStatus(USART2,USART_IT_RXNE)!= RESET)//串口接收中断
////	{
////		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
////	  Res =USART_ReceiveData(USART2);	//读取接收到的数据	
////    if((USART2_RX_STA&0x8000)==0)//接收未完成
////		{
////			if((USART2_RX_STA&0x4000)==0)//没找到帧头
////			{
////				if(Res==FRAME_HEADER)//数据帧头
////				{
////					USART2_RX_STA|=0x4000;
////					USART2_Get_Channel[USART2_RX_STA&0x3FFF]=Res;//将帧头存入数据
////					USART2_RX_STA++;
////				}
////			}
////			else//找到帧头
////			{
////					USART2_Get_Channel[USART2_RX_STA&0x3FFF]=Res;
////					USART2_RX_STA++;
////					if((USART2_RX_STA & 0x3fff) == 3)//接收到长度信息
////					{
////						length = USART2_Get_Channel[1] | ((u16)Res << 8); 
////					}
////					if((USART2_RX_STA & 0x3fff) == length + 5 + 2 + 2)
////					{
////						USART2_RX_STA|=0x8000;//接收完成了  
////					
////					}
////					if((USART2_RX_STA&0X3FFF)>(USART2_Channel_Num-1))USART2_RX_STA=0;//接收数据错误,重新开始接收									
////			}
////		}
////	}
//}







