#ifndef PID_H
#define PID_H
#include "stdint.h"
#include "stdio.h"
typedef uint32_t(*SystemTick_Fun)(void);
//��ȡʱ��
class GetTime
{
public:
    static uint8_t GetSystemTim(uint32_t(*getTick_fun)(void));
    GetTime();
protected:
    static SystemTick_Fun Get_SystemTick;
    uint32_t Dt;
    uint32_t LastTime,NowTime; 	                 
    uint8_t UpdataTimeStamp(void);
};
//PID
class PIDControl:public GetTime
{
public:

	PIDControl();
    PIDControl(float kp, float ki, float kd);
    PIDControl(float kp, float ki, float kd,float maxp, float maxi, float maxd, float maxo);
    //����pid����  kp��ki��kd��
    void SetPid(float kp, float ki, float kd);
    //��������޷�
    void SetMaxOut(float maxp, float maxi, float maxd, float maxo);
    //�������ֵ
    float PidCalc(float fpb,float set);
    //���û��ַ����С
    void SetISeparation(float);
    //�����������
    void ClearAll();
private:
    bool IFlag;//���û��ֱ�־
    float ISeparation;//��������ʹ������
    float Kp, Ki, Kd;
    float Err,LastErr,DErr,Fpb, Set;
    float Pout, Iout, Dout, Out;
    float MaxPout, MaxIout, MaxDout, MaxOut;

};

#endif
