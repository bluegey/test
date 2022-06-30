#include "init.h"
#include "stm32f10x.h"                  // Device header

SystemMonitor Real_Monitor;  //计数帧  
SystemMonitor Rate_Monitor;  //速率帧  
/**
  * @brief  监视器任务
  * @param  unused
  * @retval voif
  */

uint16_t SysErrorStatus = 0;   
void Monitor_Thread1 (void const *argument)
{
//系统状态，每一位对应一个错误，0正常，1错误
/*************************************************************************
    15  |   14  |   13   |   12     |   11     |   10   |   9   |   8    |
        |       |        |          |          |        |       |        |
--------------------------------------------------------------------------
    7   |   6   |   5    |   4      |   3      |   2     |   1   |   0   |
        |       |        |CAN_Recive|CAN_Sent  |Judgement| INA260|MPU6050|
**************************************************************************/
	
//指示灯：左上单独的led闪烁表示没有错误，常亮有错
//				连排的第一个为读取INA,第二个为CAN发送，第三个为读取裁判系统


	while(1)
	{
		memcpy(&Rate_Monitor,&Real_Monitor,sizeof(SystemMonitor));
		memset(&Real_Monitor,0,sizeof(SystemMonitor));   //将结构体内数据清零
		//读取MPU6050
		if(Rate_Monitor.IMUFrameCounter<15)
		{
		  SysErrorStatus|=(1<<0);
		}
		else
		{
		  SysErrorStatus&=~(1<<0);
		}
		 //读取INA260
		if(Rate_Monitor.INA260FrameCounter<15)
		{
		  SysErrorStatus|=(1<<1);
		}
		else
		{
		  SysErrorStatus&=~(1<<1);
		}
		//读取裁判系统数据
		if(Rate_Monitor.Judgement_Recive_FrameCounter<=20)
		{
			ReadJudgementState=NONE_STATE;
		  SysErrorStatus|=(1<<2);
		}
		else
		{
			ReadJudgementState=NORMAL_STATE;//置读取状态标志为1
		  SysErrorStatus&=~(1<<2);
		}
		//CAN发送数据
		if(Rate_Monitor.CAN_Send_FrameCounter<80)
		{
//		  SysErrorStatus|=(1<<3);
		}
		else
		{
		  SysErrorStatus&=~(1<<3);
		}
		//CAN接收数据
		if(Rate_Monitor.CAN_Recive_FrameCounter<30)
		{
//		  SysErrorStatus|=(1<<4);
		}
		else
		{
		  SysErrorStatus&=~(1<<4);
		}

		if(SysErrorStatus)  //有错误帧数据出现
		{
			LED0_ON;
		}
		else
		{
		}

	  osDelay(300);
	}

}
	
	
	
	


