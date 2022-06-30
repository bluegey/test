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
   int16_t error_now;         //���
    int16_t error_last;        //��һ�����
    int16_t error_rate;        //���仯��
    int16_t error_inter;       //������
   int16_t error_prev;        //���ϴ����
   int16_t pid_out;
    int   Inter_Max;         //������λ
    int   PID_OUT_Max;       //�����λ
} PID;


void PID_Set(PID* motor_type,	float Kp,	float Ki,	float Kd,float Inter_Max);
void PID_Control(PID* motor_type);

void PID_Reset(PID* motor_type);
void PID_Speed_Calc(PID* motor_type);

#endif





