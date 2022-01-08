/**
  ************************************* Copyright ******************************
  *
  *
  * FileName   : Motor_Task.c
  * Version    : v1.0
  * Author     : Ars Nova
  * Date       : 2021-07-16
  * Description:用于测试电机和CAN
  * Function List:
  	1. ....
  	   <version>:
  <modify staff>:
  		  <data>:
   <description>:
  	2. ...
  ******************************************************************************
 */

#include "Motor_Task.h"
#include "pwm.h"
#include "remote.h"

static void m3508_Init(Motor_3508 *M3508_Init);
static void m3508_Update(Motor_3508 *M3508_Update);

Motor_3508 M3508;
extern uint16_t Motor_Set,Angle;
extern u8 steer_flag;

extern void Motor_Task(void *pvParameters)
{
	vTaskDelay(100);
	m3508_Init(&M3508);
	while(1)
	{
	    m3508_Update(&M3508);	
	    CAN_CMD_3508(M3508.M3508_Speed[0].Give_Speed_Current,M3508.M3508_Speed[1].Give_Speed_Current,
			M3508.M3508_Speed[2].Give_Speed_Current,M3508.M3508_Speed[3].Give_Speed_Current);
		
	  	vTaskDelay(1);
	}
}
static void m3508_Init(Motor_3508 *M3508_Init)
{
	uint8_t i=0;
	
	const static fp32 Speed_Motor_Pid[3]={Motor_Speed_PID_KP,Motor_Speed_PID_KI,Motor_Speed_PID_KD};
//	const static fp32 AngleOR_Motor_Pid[3]={Motor_AngleOR_PID_KP,Motor_AngleOR_PID_KI,Motor_AngleOR_PID_KD};
//	const static fp32 AngleIR_Motor_Pid[3]={Motor_AngleIR_PID_KP,Motor_AngleIR_PID_KI,Motor_AngleIR_PID_KD};
	
	for(i=0;i < 4;i++)
	{
	M3508_Init->M3608_Date[i].get_Motor_3508_Measure = get_Motor_3508_Point(i);
	
	/*速度环PID的初始化*/
	PID_Init(&M3508_Init->M3508_Speed[i].PID_SPEED,PID_POSITION,Speed_Motor_Pid,Motor_Speed_PID_MAX_Out,Motor_SPeed_PID_MAX_IOUT);
	
	/*角度环PID的初始化*/
//	PID_Init(&M3508_Init->M3508_Angle[i].PID_Outer,PID_POSITION,AngleOR_Motor_Pid,Motor_AngleOR_PID_MAX_Out,Motor_AngleOR_PID_MAX_IOUT);
//	PID_Init(&M3508_Init->M3508_Angle[i].PID_Inner,PID_POSITION,AngleIR_Motor_Pid,Motor_AngleIR_PID_MAX_Out,Motor_AngleIR_PID_MAX_IOUT);
	}	
}

static void m3508_Update(Motor_3508 *M3508_Update)
{
	uint8_t i= 0;
	/*速度模式*/
	for(i=0;i<4;i++)//反馈
	{
	M3508_Update->M3508_Speed[i].Motor_Speed_Fdb=M3508_Update->M3608_Date->get_Motor_3508_Measure[i].speed_rpm ;
	}
	for(i=0;i<4;i++)//期望
	{
	M3508_Update->M3508_Speed[i].Motor_Speed_Set=Motor_Set;
	}
	for(i=0;i<4;i++)//速度模式下的输出
	{
	M3508_Update->M3508_Speed[i].Give_Speed_Current=PID_Calc(&M3508_Update->M3508_Speed[i].PID_SPEED,M3508_Update->M3508_Speed[i].Motor_Speed_Fdb,M3508_Update->M3508_Speed[i].Motor_Speed_Set);
	}

}
u32 annn=0;
void Steer_Task(void *pvParameters)
{
 TIM4_Int_Init(19999,71);
while(1)
{
			if(steer_flag==1)
			{
//			 TIM3_Int_Init(19999,71);
			}
//   if(annn%2==0)
//	 { GPIO_SetBits(GPIOA,GPIO_Pin_15);	 GPIO_SetBits(GPIOB,GPIO_Pin_5);	}
//	 else if(annn%2==1)
//	 {  GPIO_ResetBits(GPIOA,GPIO_Pin_15);
//	 GPIO_ResetBits(GPIOB,GPIO_Pin_5);}
			TIM_SetCompare1(TIM4,Angle*6.95+17500);
  vTaskDelay(1);
}

}





