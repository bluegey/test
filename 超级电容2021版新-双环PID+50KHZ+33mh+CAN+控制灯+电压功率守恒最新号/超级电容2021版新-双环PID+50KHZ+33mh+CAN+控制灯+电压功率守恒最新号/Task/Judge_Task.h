#ifndef _JUDGE_TASK_H_
#define _JUDGE_TASK_H_

#include "stm32f10x.h"                  // Device header
#include "judgement_info.h"
#include "protocol.h"

typedef enum
{
	NONE_STATE=0,//未读取到裁判系统数据状态
	NORMAL_STATE,//正常读取状态
}ReadJudgementState_e;

extern	ReadJudgementState_e	ReadJudgementState;
extern client_show_data_t Client_Show_Data;
extern client_show_graph_t client_show_graph;
void Judge_DataVerify(u8 *Buff);
void Send_FrameData(judge_data_id_e cmdid, uint8_t * pchMessage,uint8_t dwLength);
void Send_Judge_Update(void);

#endif


