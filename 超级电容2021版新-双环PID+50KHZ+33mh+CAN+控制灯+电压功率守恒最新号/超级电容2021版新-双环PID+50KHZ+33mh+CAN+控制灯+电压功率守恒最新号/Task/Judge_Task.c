#include "init.h"

extern u16 USART2_RX_STA;
client_show_data_t Client_Show_Data;
client_show_graph_t client_show_graph;
ReadJudgementState_e	ReadJudgementState=NONE_STATE;
//����ϵͳ���ݽ���
void Judge_DataVerify(u8 *Buff)
{
	Dateframe_t	*frame;
  if(Buff!=NULL)
  {
		
			frame=(Dateframe_t *)Buff;
			//������֡ͷ����֡����CRCУ��  ֡ͷCRC8  ��֡CRC16
			if(verify_crc16_check_sum((uint8_t *)frame, HEADER_LEN + CMD_LEN + frame->FrameHeader.data_length + CRC_LEN))
			{
				Real_Monitor.Judgement_Recive_FrameCounter++;
				judgement_data_handler(Buff);  //ͨ��У��������ݽ���
			}
			
		}
	
}


void Send_FrameData(judge_data_id_e cmdid, uint8_t * pchMessage,uint8_t dwLength)
{
	uint8_t i;
	uint8_t *addr;
	static Dateframe_t Frame = {.FrameHeader.sof = 0xA5};// ֡
	
	addr = (uint8_t *)&Frame.Data.client_show_data;
	Frame.CmdID = cmdid; //����

	Client_Show_Data.interactive_data.data_cmd_id=0xD180;
//	client_show_graph.interactive_data.data_cmd_id=0x0100;

	Frame.FrameHeader.data_length = dwLength;//���ݳ���
	Frame.FrameHeader.seq++; //֡��
	for(i = 0;i < dwLength;i++)
	{
		*addr++ = *pchMessage++;//���ݿ���
	}
	append_crc8_check_sum((unsigned char *)&Frame.FrameHeader,sizeof(Frame.FrameHeader));//����֡ͷУ��
	i = sizeof(Frame.FrameHeader) + sizeof(Frame.CmdID) + sizeof(Frame.CRC16) + dwLength;//����ʵ��֡�ĳ���
	append_crc16_check_sum((unsigned char *)&Frame,i); //������֡У��

  USART_Send_BUFF((uint8_t *)&Frame,i);               //֡����
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
