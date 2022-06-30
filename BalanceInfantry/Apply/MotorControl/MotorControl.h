#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
#include "sys.h"
#include "delay.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"
#define CAN_SLAVE_ID        0x01

#define CMDMOTORMODE      0x01
#define CMDRESETMODE      0x02
#define CMDZEROPOSITION   0x03

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#define PMAX 95.5f
#define PMIN -95.5f
#define VMAX 45.5f
#define VMIN -45.5f
#define KPMAX 500.0f
#define KPMIN 0.0f
#define KDMAX 5.0f
#define KDMIN 0.0f
#define TMAX 18.0f
#define TMIN -18.0f


class MotorControl
{
    private:
    TickType_t POSTick;
    TickType_t VELTick;
public:
//启动电机控制
void MotorControlStart();
//停止电机运行
void MotorControlStop();
//电机位置控制
void MotorControlPositionHandler();
//速度闭环控制
void MotorControlVelocityHandler();
//在发送电机位置零前，需要把电机的所有控制参数设置为0
void ZeroPosition();
//电机控制命令发送
void CanControlCmd(uint8_t cmd);
//数据转换
void SendControlPara(float fp, float fv, float fkp, float fkd, float kt);
//根据协议转换数据
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
};


#endif








