#include "init.h"
u8 USART2_Get_Channel[USART2_Channel_Num];

void BSP_USART2_Init(u32 BaudRate)
{
  //GPIO�˿�����
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
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  //����USART2ʱ��
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //ʱ�ӵ͵�ƽ�
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK������ʱ������ļ���->�͵�ƽ
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //ʱ�ӵڶ������ؽ������ݲ���
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //���һλ���ݵ�ʱ�����岻��SCLK���
	USART_ClockInit(USART2, &USART_ClockInitStruct);

  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�

  USART_Cmd(USART2, ENABLE);                //ʹ�ܴ���2
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
void USART2_IRQHandler(void)                	//�����жϷ������
{
	u8 Res;

	if(USART2->SR & USART_SR_ORE)//ORE����ж�
	{
		u8 com_data = USART2->DR;
	}
	
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!= RESET)//���ڽ����ж�
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	  Res =USART_ReceiveData(USART2);	//��ȡ���յ�������		
		osMessageQueuePut(UARTRX_MessageQueue,&Res,osPriorityLow1,0);//��Ϣ���з�������
		
	 }
	
}
//#include "init.h"
//u8 USART2_Get_Channel[USART2_Channel_Num];
//u8 receive_data[USART2_Channel_Num];	
//u16 USART2_RX_STA=0;       //����״̬���
//void BSP_USART2_Init(u32 BaudRate)
//{
//  //GPIO�˿�����
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
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//	USART_InitStructure.USART_BaudRate = BaudRate;//���ڲ�����
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

//  //����USART2ʱ��
//	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //ʱ�ӵ͵�ƽ�
//	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK������ʱ������ļ���->�͵�ƽ
//	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //ʱ�ӵڶ������ؽ������ݲ���
//	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //���һλ���ݵ�ʱ�����岻��SCLK���
//	USART_ClockInit(USART2, &USART_ClockInitStruct);

//  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
//	
//	 USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);//����DMA����
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
//DMA_Init(DMA1_Channel6,&DMA_Initstructure);     //����DMA   
//DMA_Cmd(DMA1_Channel6,ENABLE); 
//		
////  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
//	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//���������ж�

//  USART_Cmd(USART2, ENABLE);                //ʹ�ܴ���2
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

//void USART2_IRQHandler(void)                	//�����жϷ������
//{
// static uint32_t this_time_rx_len = 0;
//	unsigned char num=0;   
//	static u16 length;u8 Res;
//if(USART_GetITStatus(USART2,USART_IT_IDLE) == SET)    
//{    
////		  Res =USART_ReceiveData(USART2);	//��ȡ���յ�������	 
//num = USART2->SR;      
//num = USART2->DR; //��USART_IT_IDLE��־       
//DMA_Cmd(DMA1_Channel6,DISABLE);    //�ر�DMA       
//this_time_rx_len = 50 - DMA_GetCurrDataCounter(DMA1_Channel6);  
//	
//	if(this_time_rx_len==judgement_length){
//	    if((USART2_RX_STA&0x8000)==0)//����δ���
//		{
//			if((USART2_RX_STA&0x4000)==0)//û�ҵ�֡ͷ
//			{
//				if(Res==FRAME_HEADER)//����֡ͷ
//				{
//					USART2_RX_STA|=0x4000;
//					USART2_Get_Channel[USART2_RX_STA&0x3FFF]=Res;//��֡ͷ��������
//					USART2_RX_STA++;
//				}
//			}
//			else//�ҵ�֡ͷ
//			{
//					USART2_Get_Channel[USART2_RX_STA&0x3FFF]=Res;
//					USART2_RX_STA++;
//					if((USART2_RX_STA & 0x3fff) == 3)//���յ�������Ϣ
//					{
//						length = USART2_Get_Channel[1] | ((u16)Res << 8); 
//					}
//					if((USART2_RX_STA & 0x3fff) == length + 5 + 2 + 2)
//					{
//						USART2_RX_STA|=0x8000;//���������  
//					
//					}
//					if((USART2_RX_STA&0X3FFF)>(USART2_Channel_Num-1))USART2_RX_STA=0;//�������ݴ���,���¿�ʼ����									
//			}
//		}
//	}

//num = 128 -  DMA_GetCurrDataCounter(DMA1_Channel6);      //�õ������������ݸ���        
//receive_data[num] = '\0';       DMA1_Channel6->CNDTR=128;       //�������ý������ݸ���          
//DMA_Cmd(DMA1_Channel6,ENABLE);  //����DMA     
////  receive_flag = 1;           //�������ݱ�־λ��1   
//}
//	
//	
//	

////	if(USART2->SR & USART_SR_ORE)//ORE����ж�
////	{
////		u8 com_data = USART2->DR;
////	}
////	
////	if(USART_GetITStatus(USART2,USART_IT_RXNE)!= RESET)//���ڽ����ж�
////	{
////		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
////	  Res =USART_ReceiveData(USART2);	//��ȡ���յ�������	
////    if((USART2_RX_STA&0x8000)==0)//����δ���
////		{
////			if((USART2_RX_STA&0x4000)==0)//û�ҵ�֡ͷ
////			{
////				if(Res==FRAME_HEADER)//����֡ͷ
////				{
////					USART2_RX_STA|=0x4000;
////					USART2_Get_Channel[USART2_RX_STA&0x3FFF]=Res;//��֡ͷ��������
////					USART2_RX_STA++;
////				}
////			}
////			else//�ҵ�֡ͷ
////			{
////					USART2_Get_Channel[USART2_RX_STA&0x3FFF]=Res;
////					USART2_RX_STA++;
////					if((USART2_RX_STA & 0x3fff) == 3)//���յ�������Ϣ
////					{
////						length = USART2_Get_Channel[1] | ((u16)Res << 8); 
////					}
////					if((USART2_RX_STA & 0x3fff) == length + 5 + 2 + 2)
////					{
////						USART2_RX_STA|=0x8000;//���������  
////					
////					}
////					if((USART2_RX_STA&0X3FFF)>(USART2_Channel_Num-1))USART2_RX_STA=0;//�������ݴ���,���¿�ʼ����									
////			}
////		}
////	}
//}







