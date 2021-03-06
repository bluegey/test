#include "devicer_usart.h"

static uint8_t _UART4_DMA_RX_BUF[2][BSP_UART4_DMA_RX_BUF_LEN];

void USART1_Init(u32 My_BaudRate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    DMA_InitTypeDef dma;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

//	GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);

//	USART_DeInit(USART1);
//  USART_StructInit(&usart);
    usart.USART_BaudRate = My_BaudRate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_Even;				//偶校验
    usart.USART_Mode = USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    DMA_DeInit(DMA1_Channel5);
    DMA_StructInit(&dma);
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    dma.DMA_MemoryBaseAddr = (uint32_t)&_UART4_DMA_RX_BUF[0][0];
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = 30u;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;	 //禁止DMA通道设置为内存至内存传输
    DMA_Init(DMA1_Channel5, &dma);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TE, ENABLE);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Channel5, ENABLE);

    // 初始化 中断优先级
    USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);

    USART_Cmd(USART1, ENABLE);
}




uint8_t ucRxBuffer[250];
uint8_t ucRxCnt = 0;	//static
void USART1_IRQHandler(void)
{
    datarevice Buffer;
    static uint32_t this_time_rx_len = 0;
    if(USART_GetITStatus(USART1, USART_IT_IDLE)!=RESET) // 中断标志
    {
        (void)USART1->SR;
        (void)USART1->DR;
        DMA_Cmd(DMA1_Channel5, DISABLE);
        this_time_rx_len =BSP_UART4_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Channel5);
        DMA1_Channel5->CNDTR = (uint16_t)30u;     //relocate the dma memory pointer to the beginning position
//	DMA1_Channel5->CCR |= (uint32_t)(0<<0);
        DMA_Cmd(DMA1_Channel5, ENABLE);
        if(this_time_rx_len == 18u)
        {
//		USART_ClearFlag(USART1,USART_IT_IDLE);
            memcpy(Buffer.buffer,_UART4_DMA_RX_BUF[0],18u);
          
                RemoteDataPrcess(Buffer.buffer);
            
        }
    }
}


