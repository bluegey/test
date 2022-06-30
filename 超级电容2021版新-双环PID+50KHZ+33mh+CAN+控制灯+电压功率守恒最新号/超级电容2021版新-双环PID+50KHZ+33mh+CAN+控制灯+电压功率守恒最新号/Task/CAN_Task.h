#ifndef _CAN_TASK_H_
#define _CAN_TASK_H_

#include "stm32f10x.h"                  // Device header
#include "judgement_info.h"

#define INFANTRY 1
#define HERO 2
#define ENGINEER 3
#define SENTINEL 4
typedef struct
{
	int16_t Voltage;
	int16_t Current;
	int16_t Supply_Num;
	int16_t ShooterHeat_17mm;
	int16_t ShooterHeat_42mm;
	int16_t Power;
	int16_t PowerBuffer;
	u8 level;
	u8 hurt_type;
}JudgementType;

typedef struct
{
	u8 graph_operate_type;
	u8 graph_type;
	u8 graph_name[5];
	u8 graph_color;
	u8 graph_line_width;
	u16 graph_start_x;
	u16 graph_start_y;
	u16 graph_radius;
	u16 graph_dst_x;
	u16 graph_dst_y;
	uint8_t text_lengh;
	uint8_t text[30];
}Graph_Data_Type;

typedef struct
{
	int16_t ShootLevel;
	int16_t SuperCapacitorComment;
	float bullet_can_shoot;
	u8 State_Mask;
	u8 SuperCapacitorState;
	Graph_Data_Type Graph_Data ;
}SendToJudgementDataType;

extern uint8_t RobotType;
extern SendToJudgementDataType SendToJudgementData;
extern JudgementType Judgement;

void CAN1_Send_16bitMessage(u32 ID,int16_t DATA1,int16_t DATA2,int16_t DATA3,int16_t DATA4);

void CAN1_SendData_Update(void);

void CAN_Revicer(CanRxMsg* RxMessage);
void CAN_TxMssage(void);
extern int16_t bullets_num;
typedef struct
{
	uint8_t power_set;
	uint8_t flag;//1:”√2:≤ª”√
}Canrxmsg;

#endif


