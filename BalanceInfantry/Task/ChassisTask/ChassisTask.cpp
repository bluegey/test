#include "ChassisTask.h"
/**********************************************************************************
 *     ����ָ��Ϊx
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
   ChassisRemote = RemoteControlStr.GetRCData(); //��ȡң��������
   ChassisGyroscope=get_Gyro_Angle_Point();
   //����PID����
   VerticalPID.SetPid(10, 0, 0); //ֱ����
   SpeedlPID.SetPid(10, 0, 0);   //�ٶȻ�
   TurnlPID.SetPid(10, 0, 0);    //ת��
   //��������޷�
   VerticalPID.SetMaxOut(10000, 1000, 0, 10000); //ֱ����
   SpeedlPID.SetMaxOut(10000, 1000, 0, 10000);   //�ٶȻ�
   TurnlPID.SetMaxOut(10000, 1000, 0, 10000);    //ת��
}
//����ģʽ����
void ChassisTask::ChassisMOde()
{
   if (ChassisRemote->rc.s[ModeChannel_L] == RC_SW_DOWN && ChassisRemote->rc.s[ModeChannel_R] == RC_SW_DOWN)
   {
      ChassisMode = CHASSIS_RELAX; //����ģʽ
   }
   else if (ChassisRemote->rc.s[ModeChannel_L] == RC_SW_DOWN && ChassisRemote->rc.s[ModeChannel_R] == RC_SW_MID)
   {
      ChassisMode = CHASSIS_BALANCE; //ƽ��ģʽ
   }
   else if (ChassisRemote->rc.s[ModeChannel_L] == RC_SW_DOWN && ChassisRemote->rc.s[ModeChannel_R] == RC_SW_UP)
   {
      ChassisMode = CHASSIS_MANUAI; //�ֶ�ģʽ
   }
   else
      return;
}
//ƽ��ģʽ
 void ChassisTask::VerticalMode()
 {
      VerticalPID.PidCalc(ChassisGyroscope->ROLL,0);

 }
//������Ϊ����
void ChassisTask::ChassisBehaviorUpdata()
{
}
