#include "Can_Receive.h"

#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//云台电机数据读取
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
{                                                                                          \
	(ptr)->last_ecd = (ptr)->ecd;                                                          \
	(ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
	(ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
	(ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
	(ptr)->temperate = (rx_message)->Data[6];                                              \
}
motor_measure_t Motor_3508[4];
Encoder_process_t Encoder_Motor_3508[4];
void USB_LP_CAN1_RX0_IRQHandler(void)//
{
    ITStatus Status;
	CanRxMsg CanRxMessage;
    Status=CAN_GetITStatus(CAN1,CAN_IT_FMP0);//指定中断发生与否
    if(SET == !RESET)
    {
        CAN_Receive(CAN1,CAN_FIFO0,&CanRxMessage);
        CAN_ClearFlag(CAN1,CAN_IT_FMP0);
        switch(CanRxMessage.StdId)
        {
        case CAN_3508_M1_ID :
        case CAN_3508_M2_ID :
        case CAN_3508_M3_ID :
        case CAN_3508_M4_ID :
			
        {
			static uint8_t i = 0;
            i = CanRxMessage.StdId - CAN_3508_M1_ID;
            get_motor_measure(&Motor_3508[i],&CanRxMessage);
			EncoderProcess3508(&Encoder_Motor_3508[i],&Motor_3508[i]);
            break;
        }
        }
    }
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Motor_3508_Point(uint8_t i)
{
    return &Motor_3508[(i & 0x03)];
}
//返回底盘电机编码器变量地址，通过指针方式获取原始数据
const Encoder_process_t *get_Motor_3508_Encoder_Measure_Point(uint8_t i)
{
    return &Encoder_Motor_3508[(i & 0x03)];
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
		//计算得到连续的编码器输出值
	     v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	     v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
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


void CAN_CMD_3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = 0X200;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CAN1, &TxMessage);
}

