/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       Tx2_task.c/h
  * @brief      上位机通信任务，视觉数据处理
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020     		czw              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */
#include "Tx2_task.h"
#include "detect_task.h"
#include "math.h"
extern const unsigned char CRC8_TAB_UI[256];
void PcDataClean(unsigned  char * pData, int num);
uint8_t PcDataCheck( uint8_t *pData );
static void UART_PutChar(USART_TypeDef* USARTx, u8 ch);
static void Send_to_PC(USART_TypeDef* USARTx, Send_Tx2_t *SEND_PC);
static void Interactive_data_update(Inter_Data_t *Inter_measure);
static PC_Ctrl_Union_t PcData;
PC_Send_data Data_send;
extern QueueHandle_t TxCOM6;
extern QueueHandle_t RxCOM6;
extern u8 GFlag_state;
int QFlag_state, last_QFlag_state, last_GFlag_state;
extern float re_yaw_absolute_angle;
extern float re_pitch_relative_angle;
Send_Tx2_t TX_vision_Mes;
Inter_Data_t InterUpdate;
uint32_t TxTaskStack;
uint32_t sendTaskStack;
unsigned char  Forecast_or_not=0;
int tim=0;

void Tx2_task(void  *pvParameters)
{
    DataRevice Buffer;

    while(1)
    {
		xQueueReceive(TxCOM6,&Buffer,portMAX_DELAY);

		memcpy(PcData.PcDataArray,Buffer.buffer,MINIPC_FRAME_LENGTH);
		InterUpdate.now_pc_pitch_ref=PcData.PcDate .angle_pitch ;
		InterUpdate.now_pc_yaw_ref =PcData.PcDate .angle_yaw ;
		if(InterUpdate.last_pc_yaw_ref !=InterUpdate.now_pc_yaw_ref ||InterUpdate.last_pc_pitch_ref !=InterUpdate.now_pc_pitch_ref)
			Forecast_or_not=1;
		InterUpdate.last_pc_yaw_ref=InterUpdate.now_pc_yaw_ref;
		InterUpdate.last_pc_pitch_ref=InterUpdate.now_pc_pitch_ref;
		TxTaskStack = uxTaskGetStackHighWaterMark(NULL);
//		vTaskDelay(1);
    }
}
//float d2,er[1000];
//int d1=0;
void Interactive_task(void  *pvParameters)
{
    while(1)
    {
//       d1=xTaskGetTickCount();
//			er=d1-d2;
//			d2=d1;
//			d1++;
        TX_vision_Mes.yaw = re_yaw_absolute_angle; //获取yaw
        TX_vision_Mes.pitch = re_pitch_relative_angle; //获取pitch
        TX_vision_Mes.shoot_speed = InterUpdate.GameRobot_State_measure->shooter_id1_17mm_speed_limit; //获取速度
        Interactive_data_update(&InterUpdate);//颜色
        Send_to_PC(USART6, &TX_vision_Mes);	
//			detect_task();

//			er[d1]=TX_vision_Mes.yaw ;
//		if(d1==999)
//		{
//		d1=0;
//		}
			sendTaskStack = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(5);
				
    }
}
//视觉交互数据更新
static void Interactive_data_update(Inter_Data_t *Inter_measure)
{
    Inter_measure->GameRobot_State_measure = get_game_robot_state_t();
    Inter_measure->gameshoot_state_measure = get_shoot_data_t();

    if (Inter_measure->GameRobot_State_measure->robot_id > 10)
    {
        TX_vision_Mes.color = BLUE;//己方红色
    }
    else
    {
        TX_vision_Mes.color = RED;//己方蓝色
    }
}

//返回PC接收端变量地址，通过指针方式获取原始数据-----通过返回数据控制云台
const PC_Ctrl_Union_t *get_PC_Ctrl_Measure_Point(void)
{
    return &PcData;
}

//PC数据校验函数
uint8_t PcDataCheck( uint8_t *pData )
{
    if((pData[0] == 0xA5) && (pData[MINIPC_FRAME_LENGTH - 1] == 0x5A))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
//数据清零
void PcDataClean(unsigned  char * pData, int num)
{
    if(pData == NULL)
    {
        return;
    }

    int i = 0;

    for(i = 0; i < num; i++)
    {
        pData[i] = 0;
    }
}
unsigned char le[12];
//unsigned char hhh[254];
static void Send_to_PC(USART_TypeDef* USARTx, Send_Tx2_t *TXmessage)
{
    unsigned char crc = 0;
    unsigned char *TX_data;
    TX_data = (unsigned char*)TXmessage;
    crc = get_crc8_check_sum(TX_data, 11, 0xff); //CRC校验
    TXmessage->CRC8 = crc;
    //数据发送
    UART_PutChar(USARTx, 0x5a);//帧头
   
    for (int i = 0; i < 12; i++)
    {
        UART_PutChar(USARTx, *TX_data); //数据 + crc		
        TX_data++;

    }
//   detect_task();
//	UART_PutChar(USARTx,0xa5);//帧尾
}

static void UART_PutChar(USART_TypeDef* USARTx, u8 ch)
{
    USART_SendData(USARTx, (u8)ch);

    while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
}















