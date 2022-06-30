#include "Init.h"

SuperCapacitor_State_e SuperCapacitor_State;
int i=0;
int16_t SuperCapacitorState;
void PowerSupply_Task()
{
	i++;
		if(SendToJudgementData.SuperCapacitorComment == SuperCapacitor_ON)
		{
//				 if(Judgement.PowerBuffer<=50)
//		{
//		Charge_Disable;
////			osDelay(200);
//		}else
		{
			Charge_Enable;
		}		
		Battery_Disable;
			SuperCapacitor_Enable;
		SuperCapacitorState=1; 
		}
		else if(SendToJudgementData.SuperCapacitorComment == SuperCapacitor_Charge)
		{
			Battery_Enable;
			SuperCapacitor_Disable;
				 if(Judgement.PowerBuffer<=40)
		{
		Charge_Disable;
//			osDelay(200);
		}else
		{
			Charge_Enable;
		}			

		SuperCapacitorState=2;
		}
		else
		{ 
			Charge_Disable;
			SuperCapacitor_Disable;
			Battery_Enable;
	SuperCapacitorState=0;
		}
}
