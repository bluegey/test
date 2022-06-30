#ifndef CANRECEIVE_H
#define CANRECEIVE_H
#include "can.h"
#include "stm32f4xx.h"
// #define GET(类型名,变量名,函数后缀名)    private:类型名 变量名;\//
// public:\
// TYPE Get##函数后缀名(){return 变量名;}

//宏定义单个电机结构体对象，方便使用
#define GET(TYPE, VARIABLE, SUFFIX) \
private:                            \
    TYPE VARIABLE;                  \
                                    \
public:                             \
    TYPE Get##SUFFIX() { return VARIABLE; }
//宏定义多个电机结构体
#define GETARR(TYPE, NUM, VARIABLE, SUFFIX) \
private:                                    \
    TYPE VARIABLE[NUM];                     \
                                            \
public:                                     \
    TYPE *Get##SUFFIX() { return VARIABLE; }

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x206,
    CAN_PIT_MOTOR_ID = 0x205,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    CAN_TRIGGER_MOTOR_ID = 0x201,
    CAN_FRICTION_right_ID = 0x202,
    CAN_FRICTION_left_ID = 0x203,

} CanMsgId;
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int32_t count;
    int32_t all_ecd;
} MotorMeasure;
class CanReceive
{
private:
    //宏定义多个电机结构体对象，方便统一调用  (类型名,对象个数，变量名,函数后缀名)
    GETARR(static MotorMeasure, 2, MotorChassis, MotorChassis);   //底盘电机
    GETARR(static MotorMeasure, 2, MotorFriction, MotorFriction); //摩擦轮
    //宏定义单个电机结构体对象，方便统一调用   (类型名,变量名,函数后缀名)
    GET(static MotorMeasure, MotorTrigger, MotorTrigger); //
    GET(static MotorMeasure, MotorYaw, MotorYaw);
    GET(static MotorMeasure, MotorPitch, MotorPitch);

public:
    // can1接收函数
    static void CAN1_hook(CanRxMsg *rx_message);
    // can2接收函数
    static void CAN2_hook(CanRxMsg *rx_message);
    // can1相关发送函数
    static void CAN1Send(uint32_t ID, uint8_t *buf, uint8_t len); //
    static void CAN1SendMotor(uint32_t ID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    // can1相关发送函数
    static void CAN2Send(uint32_t ID, uint8_t *buf, uint8_t len);
    static void CAN2SendMotor(uint32_t ID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    static void GetMotorMeasure(MotorMeasure &, CanRxMsg *);
};
// extern CanReceive CanReceiveStr;
#endif
