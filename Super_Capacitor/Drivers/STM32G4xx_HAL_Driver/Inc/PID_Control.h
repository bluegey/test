#ifndef __PID_CONTROL_H__
#define __PID_CONTROL_H__
//#include <stm32f10x.h>
#include "main.h"


enum
{
  LLAST = 0,
  LAST,
  NOW,
  POSITION_PID,
  DELTA_PID,
};
typedef struct pid_t
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3];

  float pout;
  float iout;
  float dout;
  float out;

  float input_max_err;    //input max err;
  float err_deadband;     //error  deadband;
  float output_deadband;  //output deadband; 
  
  uint32_t pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;
	float proportion;        //ռ����ܱ�
	float final_out;				 //���������������趨ֵ
	s8    out_sign;					 //����ʱ��������־
  void (*f_param_init)(struct pid_t *pid, 
                       uint32_t      pid_mode,
                       uint32_t      max_output,
                       uint32_t      inte_limit,
                       float         p,
                       float         i,
                       float         d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} pid_t;

//#define Inter_Max   500
typedef struct PID_PARAMETER
{
	float ref;
	float fdb;
	float Kp;
	float Ki;
	float Kd;
	float error_now;         //���
	float error_last;        //��һ�����
	float error_rate;        //���仯��
	float error_inter;       //������
	float error_prev;        //���ϴ����
	float pid_out;
	float proportion;        //ռ����ܱ�
	float final_out;				 //���������������趨ֵ
	s8    out_sign;					 //����ʱ��������־
	float Inter_Max;				//�����޷�
}PID;


extern PID pide;

typedef struct te
{
	int PID_ref;
	int PID_fdb;
	int PID_errol;
}pidtest;



void PID_Set(PID* motor_type,	float Kp,	float Ki,	float Kd,float Inter_Max);


void PID_Control(PID* motor_type); 
void PID_Reset(PID* motor_type);
void PID_Speed_Calc(PID* motor_type);
void PID_Control_Motor(PID* motor_type);


void PID_struct_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd);

float pid_calc(pid_t *pid, float fdb, float ref);
float pid_calc_err_limit(pid_t *pid, float fdb, float ref);
void pid_reset(pid_t *pid, float kp, float ki, float kd);   
float ANGLE_TO_RADIAN(float *angle);
float loop_fp32_constrain(float Input, float minValue, float maxValue);
#endif
