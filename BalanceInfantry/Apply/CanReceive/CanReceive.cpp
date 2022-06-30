#include "CanReceive.h"
// CanReceive CanReceiveStr;

MotorMeasure CanReceive::MotorChassis[2] = {0, 0};
MotorMeasure CanReceive::MotorFriction[2] = {0, 0};
MotorMeasure CanReceive::MotorTrigger = {0};
MotorMeasure CanReceive::MotorYaw = {0};
MotorMeasure CanReceive::MotorPitch = {0};
//获取电机数据
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        if (((ptr)->ecd - (ptr)->last_ecd) > 4096)                                             \
        {                                                                                      \
            (ptr)->count--;                                                                    \
        }                                                                                      \
        else if (((ptr)->ecd - (ptr)->last_ecd) < -4096)                                       \
        {                                                                                      \
            (ptr)->count++;                                                                    \
        }                                                                                      \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
        (ptr)->all_ecd = (ptr)->count * 8192 + (ptr)->ecd;                                     \
    }

void CanReceive::CAN1_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        //处理电机ID号
        i = rx_message->StdId - CAN_3508_M1_ID;
        //处理电机数据宏函数
        get_motor_measure(&CanReceive::MotorChassis[i], rx_message);
        break;
    }
    case CAN_YAW_MOTOR_ID:
    {
        get_motor_measure(&MotorYaw, rx_message);

        break;
    }
    case CAN_PIT_MOTOR_ID:
    {
        get_motor_measure(&MotorPitch, rx_message);

        break;
    }
    default:
        break;
    }
}

void CanReceive::CAN2_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    case CAN_TRIGGER_MOTOR_ID:
    {
        get_motor_measure(&MotorTrigger, rx_message);
        break;
    }
    case CAN_FRICTION_right_ID:
    case CAN_FRICTION_left_ID:
    {
        static uint8_t i = 0;
        //处理电机ID号
        i = rx_message->StdId - CAN_FRICTION_right_ID;
        //处理电机数据宏函数
        get_motor_measure(&CanReceive::MotorFriction[i], rx_message);
        break;
    }
    default:
        break;
    }
}

void CanReceive::GetMotorMeasure(MotorMeasure &motormeasure, CanRxMsg *rxmessage)
{
    if (motormeasure.ecd - motormeasure.last_ecd > 4096)
    {
        motormeasure.count--;
    }
    else if ((motormeasure.ecd - motormeasure.last_ecd) < -4096)
    {
        motormeasure.count++;
    }
    motormeasure.last_ecd = motormeasure.ecd;
    motormeasure.ecd = (uint16_t)(rxmessage->Data[0] << 8 | rxmessage->Data[1]);
    motormeasure.speed_rpm = (uint16_t)(rxmessage->Data[2] << 8 | rxmessage->Data[3]);
    motormeasure.given_current = (uint16_t)(rxmessage->Data[4] << 8 | rxmessage->Data[5]);
    motormeasure.temperate = rxmessage->Data[6];
    motormeasure.all_ecd = motormeasure.count * 8192 + motormeasure.ecd;
}

void CanReceive::CAN1Send(uint32_t ID, uint8_t *buf, uint8_t len)
{
    CanTxMsg CANTXMessage;
    CANTXMessage.StdId = ID;
    CANTXMessage.IDE = CAN_Id_Standard;
    CANTXMessage.RTR = CAN_RTR_Data;
    CANTXMessage.DLC = len;
    for (int i = 0; i < len; i++)
    {
        CANTXMessage.Data[i] = *(buf++);
    }
    CAN_Transmit(CAN1, &CANTXMessage);
}

void CanReceive::CAN2Send(uint32_t ID, uint8_t *buf, uint8_t len)
{
    CanTxMsg CANTXMessage;
    CANTXMessage.StdId = ID;
    CANTXMessage.IDE = CAN_Id_Standard;
    CANTXMessage.RTR = CAN_RTR_Data;
    CANTXMessage.DLC = len;
    for (int i = 0; i < len; i++)
    {
        CANTXMessage.Data[i] = *(buf++);
    }
    CAN_Transmit(CAN2, &CANTXMessage);
}

void CanReceive::CAN1SendMotor(uint32_t ID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;

    TxMessage.StdId = ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
    CAN_Transmit(CAN1, &TxMessage);
}

void CanReceive::CAN2SendMotor(uint32_t ID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;

    TxMessage.StdId = ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
    CAN_Transmit(CAN2, &TxMessage);
}


#ifdef __cplusplus
extern "C"
{
#endif

    void CAN1_RX0_IRQHandler(void)
    {
        static CanRxMsg rx1_message;

        if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
        {
            //        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
            CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
            CanReceive::CAN1_hook(&rx1_message);
        }
    }

    void CAN2_RX1_IRQHandler(void)
    {
        static CanRxMsg rx2_message;

        if (CAN_GetITStatus(CAN2, CAN_IT_FMP1) != RESET)
        {
            //        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
            CAN_Receive(CAN2, CAN_FIFO1, &rx2_message);
            CanReceive::CAN2_hook(&rx2_message);
        }
    }

#ifdef __cplusplus
}
#endif
