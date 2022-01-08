#include "RC.h"
#include "stdio.h"
#include "usart.h"

//static uint8_t SBUS_rx_buf[2][36];
extern  uint8_t _USART1_DMA_RX_BUF[2][BSP_USART1_DMA_RX_BUF_LEN];
RC_ctrl_t rc_ctrl;
const RC_ctrl_t *get_RC_DATD(void)
{
   return &rc_ctrl;
}
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff;        //!< Channel 2
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
u8 resss[18],i=0;
void USART1_IRQHandler(void) 
{
	    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//检查指定的USART中断是否发生  接受中断
    {

        resss[i]=USART_ReceiveData(USART1);//
			  i++;
    }
		if(i==17)
		{i=0;}
		if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET)
		{
				(void)USART1->SR;
        (void)USART1->DR;
			 static uint16_t this_time_rx_len = 0;
//       USART_ReceiveData(USART1);
			 DMA_Cmd(DMA1_Channel5, DISABLE);
			this_time_rx_len=BSP_USART1_DMA_RX_BUF_LEN-DMA_GetCurrDataCounter(DMA1_Channel5);

//			if(DMA_GetCurrDataCounter(DMA1_Channel5)==0)
//			{
			SBUS_TO_RC(&_USART1_DMA_RX_BUF[0][0], &rc_ctrl);
//			}
			DMA1_Channel5->CNDTR=BSP_USART1_DMA_RX_BUF_LEN;//relocate the dma memory pointer to the beginning position
			 
			
//			DMA_ClearFlag(DMA1_FLAG_TC4);
			DMA_Cmd(DMA1_Channel5, ENABLE);


		}






}

