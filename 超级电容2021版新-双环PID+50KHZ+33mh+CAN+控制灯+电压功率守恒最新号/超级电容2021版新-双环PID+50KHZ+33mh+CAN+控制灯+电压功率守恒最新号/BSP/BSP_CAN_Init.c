#include "BSP_CAN_Init.h"

#include "init.h"
CanRxMsg RXMsg;  
CanTxMsg TXMsg; //can 发送邮箱

void BSP_Can_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	CAN_InitTypeDef CAN_InitStruct;
	CAN_FilterInitTypeDef	CAN_FilterInitStruct;

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_11;//RX
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPU;//上拉
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12;//TX
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//推挽复用
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	/*CAN_InitStructure寄存器初始化*/
	CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStruct);
	
	/*CAN_InitStructure单元初始化*/
	CAN_InitStruct.CAN_TTCM=DISABLE;//非时间触发通信模式
	CAN_InitStruct.CAN_TXFP=ENABLE;//优先级由报文标识符决定
	CAN_InitStruct.CAN_RFLM=ENABLE;//报文锁定，新的不覆盖旧的
	CAN_InitStruct.CAN_AWUM=ENABLE;//睡眠模式通过软件唤醒
	CAN_InitStruct.CAN_ABOM=DISABLE;//软件自动离线管理
	CAN_InitStruct.CAN_NART=DISABLE;//禁止报文自动传送
	CAN_InitStruct.CAN_Mode=CAN_Mode_Normal;//正常模式
	CAN_InitStruct.CAN_SJW=CAN_SJW_1tq;//
	CAN_InitStruct.CAN_BS1=CAN_BS1_6tq;//
	CAN_InitStruct.CAN_BS2=CAN_BS2_5tq;//
	CAN_InitStruct.CAN_Prescaler=3;//CAN BaudRate 36/(1+BS1+BS2)/3=1Mbps
	CAN_Init(CAN1,&CAN_InitStruct);
	
	/*************************************初始化过滤器组************************************/
	//CAN1的过滤器组从0-13
	CAN_FilterInitStruct.CAN_FilterNumber = 0;													//选择过滤器组0
	CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;				//设置标识符列表模式
	CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;				//设置过滤器组0的位宽为32
	CAN_FilterInitStruct.CAN_FilterIdHigh = 0x0000;											//设置所有过滤标识符ID都被允许
	CAN_FilterInitStruct.CAN_FilterIdLow = 0x0000;										
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh = 0x0000;									//设置所有的ID都为不用关心，都不被过滤
	CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;									
	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;									//给该过滤器分配一个FIFO0
	CAN_FilterInitStruct.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStruct);															//滤波器初始化

	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//邮箱0中断
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);	//发送中断
	CAN_ITConfig(CAN1,CAN_IT_ERR,DISABLE);
}
/*************************************************************************
描述：CAN1的接收中断函数
*************************************************************************/

void USB_LP_CAN1_RX0_IRQHandler(void)
{
		if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
		{  		 
			 CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
       CAN_Receive(CAN1,CAN_FIFO0,&RXMsg);    //CAN接收数据
			 CAN_Revicer(&RXMsg);
		}
}
void USB_HP_CAN1_TX_IRQHandler(void)
{
		if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
		{
			Real_Monitor.CAN_Send_FrameCounter++;
			
			CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
		}
}

