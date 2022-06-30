#include "ChassisTask.h"
/**********************************************************************************
 *     串口指向为x
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
***********************************************************************************/

void Chassis_task(void *pvParameters)
{
   ChassisTask ChassisTaskStu;
   ChassisTaskStu.ChassisTaskInit();
   while (1)
   {
      vTaskDelay(1);
   }
}

void ChassisTask::ChassisTaskInit()
{
   ChassisRemote = RemoteControlStr.GetRCData(); //获取遥控器数据
   ChassisGyroscope=get_Gyro_Angle_Point();
   //设置PID参数
   VerticalPID.SetPid(10, 0, 0); //直立环
   SpeedlPID.SetPid(10, 0, 0);   //速度环
   TurnlPID.SetPid(10, 0, 0);    //转向环
   //设置输出限幅
   VerticalPID.SetMaxOut(10000, 1000, 0, 10000); //直立环
   SpeedlPID.SetMaxOut(10000, 1000, 0, 10000);   //速度环
   TurnlPID.SetMaxOut(10000, 1000, 0, 10000);    //转向环
}
//底盘模式设置
void ChassisTask::ChassisMOde()
{
   if (ChassisRemote->rc.s[ModeChannel_L] == RC_SW_DOWN && ChassisRemote->rc.s[ModeChannel_R] == RC_SW_DOWN)
   {
      ChassisMode = CHASSIS_RELAX; //无力模式
   }
   else if (ChassisRemote->rc.s[ModeChannel_L] == RC_SW_DOWN && ChassisRemote->rc.s[ModeChannel_R] == RC_SW_MID)
   {
      ChassisMode = CHASSIS_BALANCE; //平衡模式
   }
   else if (ChassisRemote->rc.s[ModeChannel_L] == RC_SW_DOWN && ChassisRemote->rc.s[ModeChannel_R] == RC_SW_UP)
   {
      ChassisMode = CHASSIS_MANUAI; //手动模式
   }
   else
      return;
}
//平衡模式
 void ChassisTask::VerticalMode()
 {
      VerticalPID.PidCalc(ChassisGyroscope->ROLL,0);

 }
//底盘行为更新
void ChassisTask::ChassisBehaviorUpdata()
{
}
