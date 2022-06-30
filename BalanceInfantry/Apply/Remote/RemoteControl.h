#ifndef REMOTECONTROL_H
#define REMOTECONTROL_H
#include "stdint.h"
#include "stdio.h"
#include "sys.h"

//S1 S2 拨杆
#define 	ModeChannel_R 		0
#define 	ModeChannel_L 		1

#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

//中断优先级
#define RC_NVIC 7
//遥控器偏移
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
//缓存长度 为接收数据2倍防止溢出
#define SBUS_RX_BUF_NUM 36u
//数据接收长度
#define RC_FRAME_LENGTH 18u
//遥控器结构体
typedef __packed struct
{
    __packed struct
    {
        int16_t ch[5];
        char s[2];
    } rc;
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    __packed struct
    {
        uint16_t v;
    } key;

} RC_ctrl_t;

class RemoteControl
{
private:
    //一辆小车只能搭配一个遥控器，故将其设为静态变量为该类对象共享
    static RC_ctrl_t RcCtrl;
    static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

public:
    //初始化
    static void RCInit();
    //数据解算
    static void SbusToRc(volatile const uint8_t *sbus_buf);
    //返回遥控器数据
    static const RC_ctrl_t* GetRCData();
//    //设置转化数据
//    static RC_ctrl_t& SetRCDatd(); 
    //返回buffer0数据
    static uint8_t *GetSbusRxBuf0();
    //返回buffer1数据
    static uint8_t *GetSbusRxBuf1();
    //复位
    void RCRestart(uint16_t dma_length);
    //
};
extern RemoteControl RemoteControlStr;
#endif
