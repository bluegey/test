/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���_____������̨��������->CAN1_____�������->CAN2______
  * @note       ���ļ�����freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020		     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */

#include "CAN_Receive.h"
//#include "detect_task.h"

#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1
#define SHOOT_CAN CAN2

//���̵�����ݶ�ȡ
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
			  if(((ptr)->ecd-(ptr)->last_ecd )>4096)                                                 \
			  { (ptr)->count--;}                                                                     \
	       else if(((ptr)->ecd-(ptr)->last_ecd )<-4096)                                          \
			   {(ptr)->count++;}                                                                     \
		     (ptr)->last_ecd = (ptr)->ecd;                                 	                       \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
				 (ptr)->all_ecd =(ptr)->count*8192+(ptr)->ecd;                                         \
    }

//��̨������ݶ�ȡ
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }


//ͳһ����can���պ���
static void CAN1_hook(CanRxMsg *rx_message);
static void CAN2_hook(CanRxMsg *rx_message);
//�����������
motor_measure_t motor_yaw, motor_pit, motor_trigger, motor_friction[2], motor_chassis[4];
//������������ģ��
Super_power_t Super_power;
//��������������
Encoder_process_t Encoder_friction[2], Encoder_chassis[4], Encoder_pit, Encoder_yaw, Encoder_trigger;
//��̨���ͽṹ��
static CanTxMsg GIMBAL_TxMessage;


//can1�ж�
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
//        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN1_hook(&rx1_message);
    }
}

//can2�ж�
void CAN2_RX1_IRQHandler(void)
{
    static CanRxMsg rx2_message;

    if (CAN_GetITStatus(CAN2, CAN_IT_FMP1) != RESET)
    {
//        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &rx2_message);
        CAN2_hook(&rx2_message);
    }
}

//ͳһ����can�жϺ��������Ҽ�¼�������ݵ�ʱ�䣬��Ϊ�����ж�����
static void CAN2_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
        case CAN_TRIGGER_MOTOR_ID:
        {
            //���������ݺ꺯��
            get_motor_measure(&motor_trigger, rx_message);
            EncoderProcess3508(&Encoder_trigger, &motor_trigger);
            //��¼ʱ��
//            DetectHook(TriggerMotorTOE);
            break;
        }

        case CAN_FRICTION_right_ID:
        case CAN_FRICTION_left_ID:
        {
            static uint8_t i = 0;
            //������ID��
            i = rx_message->StdId - CAN_FRICTION_right_ID;
            //���������ݺ꺯��
            get_motor_measure(&motor_friction[i], rx_message);
            EncoderProcess3508(&Encoder_friction[i], &motor_friction[i]);
            //��¼ʱ��
//            DetectHook(frictionmotorRTOE + i);
            break;
        }

        default:
        {
            break;
        }
    }
}

static void CAN1_hook(CanRxMsg *rx_message)
{
    switch(rx_message->StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
            static uint8_t i = 0;
            //������ID��
            i = rx_message->StdId - CAN_3508_M1_ID;
            //���������ݺ꺯��
            get_motor_measure(&motor_chassis[i], rx_message);
            EncoderProcess3508(&Encoder_chassis[i], &motor_chassis[i]);
            //��¼ʱ��
//            DetectHook(ChassisMotor1TOE + i);
            break;
        }

        case CAN_YAW_MOTOR_ID:
        {
            //���������ݺ꺯��
            get_gimbal_motor_measuer(&motor_yaw, rx_message);
            EncoderProcess6020(&Encoder_yaw, &motor_yaw);
            //��¼ʱ��
//            DetectHook(YawGimbalMotorTOE);
            break;
        }

        case CAN_PIT_MOTOR_ID:
        {
            //���������ݺ꺯��
            get_gimbal_motor_measuer(&motor_pit, rx_message);
            EncoderProcess6020(&Encoder_pit, &motor_yaw);
            //��¼ʱ��
//            DetectHook(PitchGimbalMotorTOE);
            break;
        }

        case 0x300:
        {
            Super_power.volt = (uint16_t)((rx_message)->Data[1] << 8 | (rx_message)->Data[0]);
            Super_power.power = (uint16_t)((rx_message)->Data[3] << 8 | (rx_message)->Data[2]);
            Super_power.current = (uint16_t)((rx_message)->Data[5] << 8 | (rx_message)->Data[4]);
        }

        default:
        {
            break;
        }


    }
}


//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
    return &motor_trigger;
}
//����Friction���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Friction_Motor_Measure_Point(uint8_t i)
{
    return &motor_friction[(i & 0x03)];
}
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
//���ص��̵��������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const Encoder_process_t *get_Chassis_Encoder_Measure_Point(uint8_t i)
{
    return &Encoder_chassis[(i & 0x03)];
}
//����Ħ�����������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const Encoder_process_t *get_Friction_Encoder_Measure_Point(uint8_t i)
{
    return &Encoder_friction[(i & 0x03)];
}
//����trigger������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const Encoder_process_t *get_Trigger_Encoder_Measure_Point(void)
{
    return &Encoder_trigger;
}
//����yaw������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const Encoder_process_t *get_Pitch_Gimbal_Encoder_Measure_Point(void)
{
    return &Encoder_pit;
}//����pit������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const Encoder_process_t *get_Yaw_Gimbal_Encoder_Measure_Point(void)
{
    return &Encoder_yaw;
}
/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
***********************************************************************************************
*/
/*
  �������������б������ܺ͵�ֵ�ļ���
*/
void EncoderProcess3508(Encoder_process_t* v, motor_measure_t *motor)
{
    int32_t temp_sum = 0;
    v->diff = motor->ecd - motor->last_ecd;

    if(v->diff < -6500)    //���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�
    {
        v->round_cnt++;
        v->ecd_raw_rate = v->diff + 8192;
    }
    else if(v->diff > 6500)
    {
        v->round_cnt--;
        v->ecd_raw_rate = v->diff - 8192;
    }
    else
    {
        v->ecd_raw_rate = v->diff;
    }

    v->rate_buf[v->buf_count++] = v->ecd_raw_rate;

    if(v->buf_count == RATE_BUF_SIZE)
    {
        v->buf_count = 0;
    }

    //�����ٶ�ƽ��ֵ
    for(uint8_t i = 0; i < RATE_BUF_SIZE; i++)
    {
        temp_sum += v->rate_buf[i];
    }

    v->filter_rate = (int32_t)(temp_sum / RATE_BUF_SIZE);
}
/*
  �������������б������ܺ͵�ֵ�ļ���
*/
void EncoderProcess6020(Encoder_process_t* v, motor_measure_t *motor)
{
    int32_t temp_sum = 0;
    v->diff = motor->ecd - motor->last_ecd;

    if(v->diff < -4096)    //���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�
    {
        v->round_cnt++;
        v->ecd_raw_rate = v->diff + 8192;
    }
    else if(v->diff > 4096)
    {
        v->round_cnt--;
        v->ecd_raw_rate = v->diff - 8192;
    }
    else
    {
        v->ecd_raw_rate = v->diff;
    }

    v->rate_buf[v->buf_count++] = v->ecd_raw_rate;

    if(v->buf_count == RATE_BUF_SIZE)
    {
        v->buf_count = 0;
    }

    //�����ٶ�ƽ��ֵ
    for(uint8_t i = 0; i < RATE_BUF_SIZE; i++)
    {
        temp_sum += v->rate_buf[i];
    }

    v->filter_rate = (int32_t)(temp_sum / RATE_BUF_SIZE);
}

/***********************************************************************************************/

//CAN ���� 0x700��ID�����ݣ�������M3508�����������IDģʽ
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

//���͵��̵����������
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
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

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

//������̨�����������revΪ�����ֽ�
void CAN_CMD_GIMBAL(int16_t pitch, int16_t yaw, int16_t shoot, int16_t rev)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (pitch >> 8);
    GIMBAL_TxMessage.Data[1] = pitch;
    GIMBAL_TxMessage.Data[2] = (yaw >> 8);
    GIMBAL_TxMessage.Data[3] = yaw;
    GIMBAL_TxMessage.Data[4] = (shoot >> 8);
    GIMBAL_TxMessage.Data[5] = shoot;
    GIMBAL_TxMessage.Data[6] = (rev >> 8);
    GIMBAL_TxMessage.Data[7] = rev;

    CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
}


//�������ģ���������
void CAN_CMD_Shoot(int16_t trigger, int16_t fri_right, int16_t fri_left)
{
    CanTxMsg SendCanTxMsg;
    SendCanTxMsg.StdId = 0x200;
    SendCanTxMsg.IDE = CAN_ID_STD;
    SendCanTxMsg.RTR = CAN_RTR_DATA;
    SendCanTxMsg.DLC = 0x08;
    SendCanTxMsg.Data[0] = trigger   >> 8;
    SendCanTxMsg.Data[1] = trigger;
    SendCanTxMsg.Data[2] = fri_right >> 8;
    SendCanTxMsg.Data[3] = fri_right;
    SendCanTxMsg.Data[4] = fri_left  >> 8;
    SendCanTxMsg.Data[5] = fri_left;
    SendCanTxMsg.Data[6] = 0;
    SendCanTxMsg.Data[7] = 0;

    CAN_Transmit(SHOOT_CAN, &SendCanTxMsg);
}

Tx_Union_data Can_Tx_Data;
void CAN_CMD_SUPERPOWER(int16_t power, int16_t i,uint16_t buffer_power)
{
    CanTxMsg SendCanTxMsg;
	  Can_Tx_Data.TX_data .power =power;
	  Can_Tx_Data.TX_data .flag =i;
	  Can_Tx_Data.TX_data .buffer_power =buffer_power;
    SendCanTxMsg.StdId = 0x222;
    SendCanTxMsg.IDE = CAN_ID_STD;
    SendCanTxMsg.RTR = CAN_RTR_DATA;
    SendCanTxMsg.DLC = 0x08;
	 for(int i=0;i<sizeof(Send_Data);i++)
	{
    SendCanTxMsg.Data[i] = Can_Tx_Data.Array_Tx_data[i];
	}
    CAN_Transmit(CHASSIS_CAN, &SendCanTxMsg);
}
/**************************************End of file************************************************/


