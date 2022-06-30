#include "RemoteControl.h"




//创建一个遥控器对象
RemoteControl RemoteControlStr;

//初始化成员
RC_ctrl_t RemoteControl::RcCtrl = {0};
uint8_t RemoteControl::SBUS_rx_buf[2][SBUS_RX_BUF_NUM] = {0};

//初始化
void RemoteControl::RCInit()
{
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // | RCC_AHB1Periph_DMA1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE); // RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // PA3  usart2 rx
    /* -------------- Configure GPIO ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO_Speed_100MHz
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        USART_DeInit(USART2);

        USART_InitStructure.USART_BaudRate = 100000;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        USART_InitStructure.USART_Mode = USART_Mode_Rx;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART2, &USART_InitStructure);

        USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

        USART_ClearFlag(USART2, USART_FLAG_IDLE);
        USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

        USART_Cmd(USART2, ENABLE);
    }

    /* -------------- Configure NVIC ---------------------------------------*/
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RC_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    /* -------------- Configure DMA -----------------------------------------*/
    {
        DMA_InitTypeDef DMA_InitStructure;
        DMA_DeInit(DMA1_Stream5);

        DMA_InitStructure.DMA_Channel = DMA_Channel_4;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR); //更新数据流
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SBUS_rx_buf[0];
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_BufferSize = SBUS_RX_BUF_NUM; //
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream5, &DMA_InitStructure);
        DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)SBUS_rx_buf[1], DMA_Memory_0);
        DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
        DMA_Cmd(DMA1_Stream5, DISABLE); // Add a disable  DMA_Cmd(DMA1_Stream1, DISABLE)
        DMA_Cmd(DMA1_Stream5, ENABLE);  //   DMA_Cmd(DMA1_Stream1, ENABLE)
    }
}
//
void RemoteControl::RCRestart(uint16_t dma_length)
{
    USART_Cmd(USART2, DISABLE);
    DMA_Cmd(DMA1_Stream5, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Stream5, dma_length);

    USART_ClearFlag(USART2, USART_FLAG_IDLE);

    DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF4);
    DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF4);
    DMA_Cmd(DMA1_Stream5, ENABLE);
    USART_Cmd(USART2, ENABLE);
}
//数据解算
void RemoteControl::SbusToRc(volatile const uint8_t *sbus_buf)
{
    if (sbus_buf == NULL)
    {
        return;
    }

    RcCtrl.rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;                              //!< Channel 0
    RcCtrl.rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;                       //!< Channel 1
    RcCtrl.rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff; //!< Channel 2
    RcCtrl.rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;                       //!< Channel 3
    RcCtrl.rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                                             //!< Switch left
    RcCtrl.rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                                        //!< Switch right
    RcCtrl.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                                          //!< Mouse X axis
    RcCtrl.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                                          //!< Mouse Y axis
    RcCtrl.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                                        //!< Mouse Z axis
    RcCtrl.mouse.press_l = sbus_buf[12];                                                        //!< Mouse Left Is Press ?
    RcCtrl.mouse.press_r = sbus_buf[13];                                                        //!< Mouse Right Is Press ?
    RcCtrl.key.v = sbus_buf[14] | (sbus_buf[15] << 8);                                          //!< KeyBoard value
    RcCtrl.rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                                       // NULL

    RcCtrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    RcCtrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    RcCtrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    RcCtrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    RcCtrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
//返回数据
const RC_ctrl_t* RemoteControl::GetRCData()
{
    return &RcCtrl;
}
////设置转化数据
//RC_ctrl_t& RemoteControl::SetRCDatd()
//{
//     return RcCtrl;
//}
//返回buffer0缓存数据
uint8_t *RemoteControl::GetSbusRxBuf0()
{
    return SBUS_rx_buf[0];
}
//返回buffer1缓存数据
uint8_t *RemoteControl::GetSbusRxBuf1()
{
    return SBUS_rx_buf[1];
}
//串口中断读取数据
#ifdef __cplusplus
extern "C"
{
#endif
    void USART2_IRQHandler(void)
    {
        if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //检查指定的USART中断是否发生  接受中断
        {
            USART_ReceiveData(USART2); //
        }
        else if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) //空闲中断
        {
            //			LED2(ON);
            static uint16_t this_time_rx_len = 0;
            USART_ReceiveData(USART2);

            if (DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0)
            {
                //重新设置DMA
                DMA_Cmd(DMA1_Stream5, DISABLE);
                this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
                DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
                DMA1_Stream5->CR |= DMA_SxCR_CT;
                //清DMA中断标志
                DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);
                DMA_Cmd(DMA1_Stream5, ENABLE);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    //处理遥控器数据
									RemoteControl::SbusToRc(RemoteControl::GetSbusRxBuf0());
                    //              RC_MODE_CONTROL(&(RemoteControl::RcCtrl));
                    //                //记录数据接收时间
                    //                DetectHook(DBUSTOE);
                }
            }
            else
            {
                //					LED2(OFF);
                //重新设置DMA
                DMA_Cmd(DMA1_Stream5, DISABLE);
                this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
                DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
                DMA1_Stream5->CR &= ~(DMA_SxCR_CT);
                //清DMA中断标志
                DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);
                DMA_Cmd(DMA1_Stream5, ENABLE);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    //处理遥控器数据
                    RemoteControl::SbusToRc(RemoteControl::GetSbusRxBuf1());
                    //               RC_MODE_CONTROL(&(RemoteControl::RcCtrl));
                    //记录数据接收时间
                    //  DetectHook(DBUSTOE);
                }
            }
            USART_ClearITPendingBit(USART2, USART_IT_IDLE);
        }
    }

#ifdef __cplusplus
}
#endif
