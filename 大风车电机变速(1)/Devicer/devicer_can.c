#include "init.h"
CanTxMsg  TxMessage;   //发送信息结构体
CanRxMsg  RxMessage;
motor_measure_t motormeaser;
Encoder_process_t motorfif;
u8	RX_data[8],i;
/*********************************************************************
*函数名称:	CANTransfer_Init
*函数功能:	CAN传输初始化
*输入参数:
*输出参数:
*********************************************************************/
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
void CANTransfer_Init(void)
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
    CAN_InitStruct.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒
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
    CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);	//发送中断
	
//	NVIC_InitStructure.NVIC_IRQChannel=USB_LP_CAN1_RX0_IRQn; 
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; 
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0; 
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE; 
//	 
//	NVIC_Init(&NVIC_InitStructure); 
}

/*************************************************************************
描述：CAN1的接收中断函数
*************************************************************************/
CanRxMsg Rx_message;
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &Rx_message);
		get_motor_measure(&motormeaser, &Rx_message); 
		EncoderProcess3508(&motorfif ,&motormeaser);
		
    }
}

/*************************************************************************
描述：CAN1的发送中断函数
*************************************************************************/
void CAN1_TX_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}


void CAN_send_2006(int16_t a)
{
    CanTxMsg tx_message;
    tx_message.StdId =0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

    tx_message.Data[0] = (uint8_t)(a>> 8);
    tx_message.Data[1] = (uint8_t)a;
    tx_message.Data[2] = (uint8_t)(a>> 8);
    tx_message.Data[3] = (uint8_t)a;
    tx_message.Data[4] = (uint8_t)(a>> 8);
    tx_message.Data[5] = (uint8_t)a;
    tx_message.Data[6] = (uint8_t)(a>> 8);
    tx_message.Data[7] = (uint8_t)a;
    CAN_Transmit(CAN1,&tx_message);
}

void EncoderProcess3508(Encoder_process_t* v ,motor_measure_t *motor)
{
	int32_t temp_sum = 0; 
		v->diff=motor->ecd-motor->last_ecd;
		if(v->diff < -6500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
		{
			v->round_cnt++;
			v->ecd_raw_rate = v->diff + 8192;
		}
		else if(v->diff > 6500)
		{
			v->round_cnt--;
			v->ecd_raw_rate = v->diff- 8192;
		}
		else
		{
			v->ecd_raw_rate = v->diff;
		}
		v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
		if(v->buf_count == RATE_BUF_SIZE)
		{
			v->buf_count = 0;
		}
		//计算速度平均值
		for(uint8_t i = 0;i < RATE_BUF_SIZE; i++)
		{
			temp_sum += v->rate_buf[i];
		}

		v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		
}