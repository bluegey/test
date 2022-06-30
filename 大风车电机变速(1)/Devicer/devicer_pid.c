#include "devicer_pid.h"


void PID_Control(PID* motor_type)
{
//  float error_position;
    motor_type->error_last=motor_type->error_now;
    motor_type->error_now = motor_type->ref - motor_type->fdb;
    motor_type->error_rate=motor_type->error_now-motor_type->error_last;
    motor_type->error_inter += motor_type->error_now;
    // limit intergration of pid
    if(motor_type->error_inter>motor_type->Inter_Max)
        motor_type->error_inter = motor_type->Inter_Max;
    if(motor_type->error_inter<-motor_type->Inter_Max)
        motor_type->error_inter = -motor_type->Inter_Max;

    motor_type->pid_out = (motor_type->Kp * motor_type->error_now + motor_type->Ki * motor_type->error_inter +	motor_type->Kd * motor_type->error_last);
}

/*********************ÔöÁ¿Ê½PID******************************/
void PID_Speed_Calc(PID* motor_type)
{

    motor_type->error_now = motor_type->ref - motor_type->fdb;

    motor_type->pid_out = motor_type->pid_out + motor_type->Kp * motor_type->error_now               //E[k]
                          - motor_type->Ki   * motor_type->error_last     //E[k-1]
                          + motor_type->Kd * motor_type->error_prev;   //E[k-2]

    motor_type->error_prev = motor_type->error_last;
    motor_type->error_last = motor_type->error_now;
}

void PID_Reset(PID* motor_type)
{
    motor_type->ref = 0;
    motor_type->fdb = 0;
    motor_type->Kp = 0;
    motor_type->Ki = 0;
    motor_type->Kd = 0;
    motor_type->error_last = 0;
    motor_type->error_now = 0;
    motor_type->error_rate = 0;
    motor_type->error_inter = 0;
    motor_type->error_prev=0;
    motor_type->pid_out = 0;
}

void PID_Set(PID* motor_type,	float Kp,	float Ki,	float Kd,float Inter_Max)
{
    motor_type->Kp = Kp;
    motor_type->Ki = Ki;
    motor_type->Kd = Kd;
    motor_type->Inter_Max=Inter_Max;
}

