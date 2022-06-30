#include "MotorControl.h"
#include "CanReceive.h"


//启动电机控制
void MotorControl::MotorControlStart()
{
    ZeroPosition();
    CanControlCmd(CMDZEROPOSITION);
    POSTick=xTaskGetTickCount();
    VELTick=xTaskGetTickCount();
}
//停止模式
void MotorControl::MotorControlStop()
{
CanControlCmd(CMDRESETMODE);
}
//电机位置控制
void MotorControl::MotorControlPositionHandler()
{


}
//电机速度闭环控制
void MotorControl::MotorControlVelocityHandler()
{


}

void MotorControl::ZeroPosition()
{
    CanControlCmd(CMDMOTORMODE);
    delay_ms(100);
    SendControlPara(0, 0, 0, 0, 0);
    delay_ms(100);
}
void MotorControl::CanControlCmd(uint8_t cmd)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    ;
    switch (cmd)
    {
    case CMDMOTORMODE:
        buf[7] = 0xFC;
        break;
    case CMDRESETMODE:
        buf[7] = 0xFD;
        break;
    case CMDZEROPOSITION:
        buf[7] = 0xFE;
        break;
    default:
        return;
    }

    CanReceive::CAN1Send(CAN_SLAVE_ID, buf, 8);
}
void MotorControl::SendControlPara(float fp, float fv, float fkp, float fkd, float kt)
{
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
    //将输入参数限制在定义范围内
    LIMIT_MIN_MAX(fp, PMIN, PMAX);
    LIMIT_MIN_MAX(fv, VMIN, VMAX);
    LIMIT_MIN_MAX(fkp, KPMIN, KPMAX);
    LIMIT_MIN_MAX(fkd, KDMIN, KDMAX);
    LIMIT_MIN_MAX(kt, TMIN, TMAX);
    //根据通信协议进行参数转换
    p = float_to_uint(fp, PMIN, PMAX, 16);
    v = float_to_uint(fv, VMIN, VMAX, 12);
    kp = float_to_uint(fkp, KPMIN, KPMAX, 12);
    kd = float_to_uint(fkd, KDMIN, KDMAX, 12);
    t = float_to_uint(kt, TMIN, TMAX, 12);
    // 根据传输协议，把数据转换为CAN命令数据字段
    buf[0] = p >> 8;
    buf[1] = p & 0xFF;
    buf[2] = v >> 4;
    buf[3] = ((v & 0xF) << 4) | (kp >> 8);
    buf[4] = kp & 0xFF;
    buf[5] = kd >> 4;
    buf[6] = ((kd & 0xF) << 4) | (t >> 8);
    buf[7] = t & 0xff;
    CanReceive::CAN1Send(CAN_SLAVE_ID, buf, 8);
}
uint16_t MotorControl::float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
