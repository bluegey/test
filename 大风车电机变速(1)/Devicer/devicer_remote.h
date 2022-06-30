#ifndef _DEVICER_REMOTE_H_
#define _DEVICER_REMOTE_H_
#include "init.h"

#define REMOTE_SWITCH_VALUE_BUF_DEEP   			16u




//to detect the action of the switch type
typedef struct RemoteSwitch_t
{
    uint8_t switch_value_raw;            // the current switch value
    uint8_t switch_value1;				  //  last value << 2 | value
    uint8_t switch_value2;				  //
    uint8_t switch_long_value; 		  //keep still if no switching
    uint8_t switch_value_buf[REMOTE_SWITCH_VALUE_BUF_DEEP];
    uint8_t buf_index;
    uint8_t buf_last_index;
    uint8_t buf_end_index;
} RemoteSwitch_t;


//遥控器结构体定义
typedef __packed struct
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    int8_t s1;
    int8_t s2;
} Remote;
//键盘鼠标结构体定义
typedef __packed struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t last_press_l;
    uint8_t last_press_r;
    uint8_t press_l;
    uint8_t press_r;
} Mouse;

typedef	__packed struct
{
    uint16_t v;
    uint16_t last_v;
} Key;

typedef __packed struct
{
    Remote rc;
    Mouse mouse;
    Key key;
} RC_Ctl_t;


typedef struct {
    int32_t raw_value;   									//编码器不经处理的原始值
    int32_t last_raw_value;								//上一次的编码器原始值
    int32_t diff;													//两次编码器之间的差值
    int32_t ecd_bias;											//初始编码器值
    int32_t round_cnt;										//圈数
    u8 first;
    int16_t speeed_rpm;//电机返回速度
    int16_t current;//转矩电流
    float ecd_angle;
} Encoder;


extern RC_Ctl_t RC_CtrlData;   	//remote control data


void SetInputMode(Remote *rc);
void CAN_recive_2006(CanRxMsg *msg);
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void M2006_set(float FM1,u8 mod);

void init_PWM_Set(void);
void LIFT_THREE(void);
void init_run(void);
void LIFT_ONE(void);

void show_mode(void);

void zhiliu(float FM1);
void diangang(float FM1);
void qigang(float FA);
void RemoteDataPrcess(uint8_t *pData);

void Selectmode(uint8_t ie);

#endif

