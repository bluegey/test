#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H
#include "sys.h"
#define  RATE_BUF_SIZE 6
typedef struct
{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;    
	int32_t diff;	
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;		
	int32_t round_cnt;
	int32_t rate_buf[RATE_BUF_SIZE]; 	//buf，for filter
	uint8_t buf_count;					//滤波更新buf用
	int32_t filter_rate;				//速度
	float ecd_angle;											//角度

} Encoder_process_t;


//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	
} motor_measure_t;

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_GIMBAL_ALL_ID = 0x1FF,
	
    CAN_TRIGGER_MOTOR_ID    = 0x201,
    CAN_FRICTION_right_ID   = 0x202, 
    CAN_FRICTION_left_ID    = 0x203,

} can_msg_id_e;

extern void CAN_CMD_3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void EncoderProcess3508(Encoder_process_t* v ,motor_measure_t *motor);
extern const Encoder_process_t *get_M3508_Encoder_Measure_Point(uint8_t i);
extern const motor_measure_t *get_Motor_3508_Point(uint8_t i);

#endif
