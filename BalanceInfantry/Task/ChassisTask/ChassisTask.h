#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "RemoteControl.h"
#include "PID.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Mputask.h"
typedef enum
{
    CHASSIS_RELAX = 0,           //地盘无力模式
    CHASSIS_BALANCE = 1,         //直立模式
    CHASSIS_MANUAI = 2,          //
    CHASSIS_FOLLOW_GIMBAL = 3,   //自动跟随云台
    CHASSIS_SEPARATE_GIMBAL = 4, //云台分离
    CHASSIS_DODGE_MODE = 5,      //小陀螺
} chassis_mode_e;                //底盘运行模式

class ChassisTask
{
private:
    const RC_ctrl_t *ChassisRemote; //获取遥控器数据指针
    const Angular_Handle *ChassisGyroscope;
    PIDControl VerticalPID;         //创建直立环PID对象
    PIDControl SpeedlPID;           //创建速度环对象
    PIDControl TurnlPID;            //创建转向环对象
    u8 ChassisMode;

public:
    //底盘初始化
    void ChassisTaskInit();
    //底盘模式设置
    void ChassisMOde();
    //平衡模式
    void VerticalMode();
    //底盘行为更新
    void ChassisBehaviorUpdata();
};

void Chassis_task(void *pvParameters);

#endif
