#ifndef __DEVICER_PID_H__
#define __DEVICER_PID_H__
#include "init.h"

typedef struct PID_PARAMETER
{
    int16_t ref;
    int16_t fdb;
    int16_t Kp;
    int16_t Ki;
    int16_t Kd;
   int16_t error_now;         //误差
    int16_t error_last;        //上一次误差
    int16_t error_rate;        //误差变化率
    int16_t error_inter;       //误差积分
   int16_t error_prev;        //上上次误差
   int16_t pid_out;
    int   Inter_Max;         //积分限位
    int   PID_OUT_Max;       //输出限位
} PID;


void PID_Set(PID* motor_type,	float Kp,	float Ki,	float Kd,float Inter_Max);
void PID_Control(PID* motor_type);

void PID_Reset(PID* motor_type);
void PID_Speed_Calc(PID* motor_type);

#endif





