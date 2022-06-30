#include "init.h"

Canrxmsg Can_power;
uint8_t RobotType;
JudgementType Judgement;
SendToJudgementDataType SendToJudgementData;
u8 robot_color;
u32 systick_ms=0;


void CAN_Revicer(CanRxMsg* RxMessage)
{
	if(RxMessage->StdId==0x222)
	{
		Can_power.power_set=RxMessage->Data[0];
		Can_power.flag=RxMessage->Data[1];
	}
}


void CAN1_Send_16bitMessage(u32 ID,int16_t DATA1,int16_t DATA2,int16_t DATA3,int16_t DATA4)
{
	CanTxMsg CAN_TxMessage;
	union SendData{
			uint8_t c[2];
			int16_t i;
	}Data1,Data2,Data3,Data4;

	Data1.i=DATA1;
	Data2.i=DATA2;
	Data3.i=DATA3;
	Data4.i=DATA4;

	CAN_TxMessage.StdId=ID;
	CAN_TxMessage.RTR=CAN_RTR_DATA;
	CAN_TxMessage.IDE=CAN_ID_STD;
	CAN_TxMessage.DLC=0x08;
	CAN_TxMessage.Data[0]=Data1.c[1];
	CAN_TxMessage.Data[1]=Data1.c[0];
	CAN_TxMessage.Data[2]=Data2.c[1];
	CAN_TxMessage.Data[3]=Data2.c[0];
	CAN_TxMessage.Data[4]=Data3.c[1];
	CAN_TxMessage.Data[5]=Data3.c[0];
	CAN_TxMessage.Data[6]=Data4.c[1];
	CAN_TxMessage.Data[7]=Data4.c[0];
	
	CAN_Transmit(CAN1,&CAN_TxMessage);
}

void CAN_TxMssage(void)
{
	CAN1_Send_16bitMessage(0x300,INAReal_Data.voltage,INAReal_Data.current,Real_Voltage*1000,0);
}

