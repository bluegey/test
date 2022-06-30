#ifndef _TRANSFER_CAN_H_
#define _TRANSFER_CAN_H_

#include "stm32f10x.h"                  // Device header

#define CAN_ID 0X300
//取绝对值
#define absolute_value(x) ((x)>0? (x):(-x))
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	
} motor_measure_t;
#define  RATE_BUF_SIZE 5
typedef struct
{
	int32_t diff;	
	int32_t round_cnt;
	int32_t ecd_raw_rate;
	int32_t rate_buf[RATE_BUF_SIZE]; 	//buf，for filter
	uint8_t buf_count;					//滤波更新buf用
	int32_t filter_rate;				//速度
} Encoder_process_t;
extern CanRxMsg rx_message;
void EncoderProcess3508(Encoder_process_t* v ,motor_measure_t *motor);
void CANTransfer_Init(void);
void CAN_send_2006(int16_t a);
#endif

