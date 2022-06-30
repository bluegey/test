#include "init.h"

extern u16 USART2_RX_STA;
client_show_data_t Client_Show_Data;
client_show_graph_t client_show_graph;
ReadJudgementState_e	ReadJudgementState=NONE_STATE;
//裁判系统数据解析
void Judge_DataVerify(u8 *Buff)
{
	Dateframe_t	*frame;
  if(Buff!=NULL)
  {
		
			frame=(Dateframe_t *)Buff;
			//将进行帧头与整帧数据CRC校验  帧头CRC8  整帧CRC16
			if(verify_crc16_check_sum((uint8_t *)frame, HEADER_LEN + CMD_LEN + frame->FrameHeader.data_length + CRC_LEN))
			{
				Real_Monitor.Judgement_Recive_FrameCounter++;
				judgement_data_handler(Buff);  //通过校验进行数据解析
			}
			
		}
	
}


void Send_FrameData(judge_data_id_e cmdid, uint8_t * pchMessage,uint8_t dwLength)
{
	uint8_t i;
	uint8_t *addr;
	static Dateframe_t Frame = {.FrameHeader.sof = 0xA5};// 帧
	
	addr = (uint8_t *)&Frame.Data.client_show_data;
	Frame.CmdID = cmdid; //命令

	Client_Show_Data.interactive_data.data_cmd_id=0xD180;
//	client_show_graph.interactive_data.data_cmd_id=0x0100;

	Frame.FrameHeader.data_length = dwLength;//数据长度
	Frame.FrameHeader.seq++; //帧数
	for(i = 0;i < dwLength;i++)
	{
		*addr++ = *pchMessage++;//数据拷贝
	}
	append_crc8_check_sum((unsigned char *)&Frame.FrameHeader,sizeof(Frame.FrameHeader));//加入帧头校验
	i = sizeof(Frame.FrameHeader) + sizeof(Frame.CmdID) + sizeof(Frame.CRC16) + dwLength;//计算实际帧的长度
	append_crc16_check_sum((unsigned char *)&Frame,i); //加入整帧校验

  USART_Send_BUFF((uint8_t *)&Frame,i);               //帧发送
}

void Send_Judge_Update(void)
{
	if(RobotType==INFANTRY)
	{
		Client_Show_Data.data1 = (float)SendToJudgementData.ShootLevel;
		Client_Show_Data.data2 =(float)bullets_num;
 		Client_Show_Data.data3 = (float)(Real_Voltage);
		Client_Show_Data.mask	 = (u8)(SendToJudgementData.State_Mask);
		
//		if(judge_rece_mesg.rfid_data.event_type>>5&1)
//		{
//			
//		client_show_graph.graphic_draw.operate_tpye=2;
//			client_show_graph.graphic_draw.graphic_tpye=6;
//			client_show_graph.graphic_draw.width=2;
//			client_show_graph.graphic_draw.start_x=960;
//			client_show_graph.graphic_draw.start_y=540;
//						client_show_graph.graphic_draw.end_x=1000;
//			client_show_graph.graphic_draw.end_y=240;
//						client_show_graph.graphic_draw.radius=10;
//			client_show_graph.graphic_draw.text_lenght=10;
//			strcpy((char *)client_show_graph.graphic_draw.text,"hajhjfha");
//		}
	}
	else if(RobotType==HERO)
	{
		Client_Show_Data.data1 = (float)((int16_t)judge_rece_mesg.real_powerheat_data.chassis_power_buffer);
		if(judge_rece_mesg.game_information.robot_level==1)
		{
			Client_Show_Data.data2 = (float)(120-judge_rece_mesg.real_powerheat_data.shooter_heat0);
		}
		else if(judge_rece_mesg.game_information.robot_level==2)
		{
			Client_Show_Data.data2 = (float)(240-judge_rece_mesg.real_powerheat_data.shooter_heat0);
		}
		else if(judge_rece_mesg.game_information.robot_level==3)
		{
			Client_Show_Data.data2 = (float)(480-judge_rece_mesg.real_powerheat_data.shooter_heat0);
		}
	}
	else
	{
		Client_Show_Data.data1 = (float)SendToJudgementData.ShootLevel;
		Client_Show_Data.data2 = (float)SendToJudgementData.SuperCapacitorComment;
//		Client_Show_Data.data3 = (float)((int16_t)judge_rece_mesg.real_powerheat_data.chassisPowerBuffer);
		Client_Show_Data.data3 = (float)(INAReal_Data.voltage/1000);
		Client_Show_Data.mask	 = (u8)(~SendToJudgementData.State_Mask);
	}
}
