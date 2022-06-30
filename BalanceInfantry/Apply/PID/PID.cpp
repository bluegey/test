#include "PID.h"
//�����ֵ
template <class T>
T Absolute(T data)
{
	return data > 0 ? data : (-data);
}
//�޷�����
template <class T>
void Limit(T &data, T max, T min)
{
	if (data > max)
	{
		data = max;
	}
	else if (data < min)
	{
		data = min;
	}
	else
	{
		return;
	}
}

GetTime::GetTime()
{
	Dt = LastTime = NowTime = 0;
}
//��ʼʱ��ָ��
SystemTick_Fun GetTime::Get_SystemTick = NULL;
//��ʼ��ϵͳʱ��
uint8_t GetTime::GetSystemTim(uint32_t (*getTick_fun)(void))
{
	if (getTick_fun != nullptr)
	{
		GetTime::Get_SystemTick = getTick_fun;
		return true;
	}
	else
	{
		return false;
	}
}
//
uint8_t GetTime::UpdataTimeStamp(void)
{

	if (Get_SystemTick == nullptr)
	{
		return false;
	}
	if (LastTime == 0)
	{
		LastTime = GetTime::Get_SystemTick();
		return true;
	}
	NowTime = GetTime::Get_SystemTick();
	if (NowTime < LastTime)
	{
		Dt = (NowTime + 0xffffffff - LastTime);
	}
	else
	{
		Dt = NowTime - LastTime;
	}
	LastTime = NowTime;
	return true;
}

PIDControl::PIDControl()
{
	Pout = Iout = Dout = Out = Kp = Ki = Kd = 0;
}
//
PIDControl::PIDControl(float mkp, float mki, float mkd)
{
	Kp = mkp;
	Ki = mki;
	Kd = mkd;
}
//
PIDControl::PIDControl(float mkp, float mki, float mkd,
					   float maxp, float maxi, float maxd, float maxo)
{
	Kp = mkp;
	Ki = mki;
	Kd = mkd;
	Pout = maxp;
	Iout = maxi;
	Dout = maxd;
	Out = maxo;
}
//���� kp ki kd
void PIDControl::SetPid(float kp, float ki, float kd)
{
	Kp = kp;
	Ki = ki;
	Kd = kd;
}
//��������޷�
void PIDControl::SetMaxOut(float maxp, float maxi, float maxd, float maxo)
{
	Pout = maxp;
	Iout = maxi;
	Dout = maxd;
	Out = maxo;
}
// PID�������
float PIDControl::PidCalc(float fpb,float set )
{
	if (UpdataTimeStamp() == false)
		return false;
	Err = set - Fpb;
	DErr = (Err - LastErr) / Dt;
	if (Err > ISeparation)
	{
		IFlag = 0;
	}
	else
	{
		IFlag = 1;
	}
	Pout = Kp * Err;
	Limit<float>(Pout, MaxPout, -MaxPout);
	Iout += (IFlag * Ki) * Err;
	Limit<float>(Iout, MaxIout, -MaxIout);
	Dout = Kd * DErr;
	Limit<float>(Iout, MaxDout, -MaxDout);
	Out = Pout + Iout + Dout;
	return Out;
}
//���û��ֿ�������
void PIDControl::SetISeparation(float data)
{
	ISeparation = data;
}
//�����������
void PIDControl::ClearAll()
{
	IFlag = 0;
	ISeparation = 0;
	Kp = Ki = Kd = 0;
	Pout = Iout = Dout = Out = 0;
	Err = LastErr = DErr = Fpb = Set = 0;
	MaxPout = MaxIout = MaxDout = MaxOut = 0;
}
