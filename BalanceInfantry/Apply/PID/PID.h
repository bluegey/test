#ifndef PID_H
#define PID_H
#include "stdint.h"
#include "stdio.h"
typedef uint32_t(*SystemTick_Fun)(void);
//获取时间
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
    //设置pid参数  kp，ki，kd，
    void SetPid(float kp, float ki, float kd);
    //设置输出限幅
    void SetMaxOut(float maxp, float maxi, float maxd, float maxo);
    //计算输出值
    float PidCalc(float fpb,float set);
    //设置积分分离大小
    void SetISeparation(float);
    //清除所有数据
    void ClearAll();
private:
    bool IFlag;//采用积分标志
    float ISeparation;//开启积分使用条件
    float Kp, Ki, Kd;
    float Err,LastErr,DErr,Fpb, Set;
    float Pout, Iout, Dout, Out;
    float MaxPout, MaxIout, MaxDout, MaxOut;

};

#endif
