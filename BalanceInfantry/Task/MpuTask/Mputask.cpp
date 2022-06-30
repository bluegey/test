#include "Mputask.h"
//创建陀螺仪对象
IMU_Data_t IMU_Data;
//方便兼容以前的代码
Angular_Handle Angular_Handler;
//
SmoothFilter_t IMU_Temp_Filter;
//
MPU9250 MPU9250Str;
//温度控制pid初始化
PIDControl MpuTemPid(IMU_Temp_PID_Kp, IMU_Temp_PID_Ki, IMU_Temp_PID_Kd, IMU_Temp_PID_POUT, IMU_Temp_PID_IOUT, IMU_Temp_PID_DOUT, IMU_Temp_PID_OUT);

/**
 * @brief  创建陀螺仪姿态解算任务
 * @retval void
 */
// IMU任务
void IMU_task(void *pvParameters)
{
  int IsCalibration = 0, Duty = 0; //IsCalibration是否校准  Duty占空比
  float MpuTemp = 0;
	Mpu9250Init();
  while (1)
  {
    if (IsCalibration == 1)
    {
      IsCalibration = 0;
      MPU9250Str.IMU_Get_Accel_Cali_Data(&IMU_Data);
    }
    else
    {
      MPU9250Str.IMU_Calc(&IMU_Data);
    }
    //获取陀螺仪温度
    MPU9250Str.MPU9250_Get_Temp(&MpuTemp);
    //对温度进行滤波处理
    IMU_Data.Temp = MPU9250Str.SmoothFilterCalc(&IMU_Temp_Filter, MpuTemp);
    // PID计算占空比调节温度
    Duty = (int)MpuTemPid.PidCalc(IMU_Data.Temp, IMU_Temp_Set);
    //设置占空比
    MPU9250Str.MPU9250_Set_Heat_PWM(Duty);
    //为了兼容之前程序
    IMU_Translate();
		vTaskDelay(1);
  }
}
// IUM任务初始化
void Mpu9250Init(void)
{

  while (MPU9250Str.MPU9250_IoInit())
  {
  }
  //初始化参数
  MPU9250Str.IMU_Data_Init(&IMU_Data);
  //滤波器初始化
  MPU9250Str.SmoothFilter_Init(&IMU_Temp_Filter, 8);
  //当误差为2时开启积分减少静态误差
  MpuTemPid.SetISeparation(2.0f);
}
//
void IMU_Translate()
{
  Angular_Handler.Pitch = IMU_Data.pitch;
  Angular_Handler.ROLL = IMU_Data.roll;
  Angular_Handler.YAW = IMU_Data.yaw;
  Angular_Handler.V_X = IMU_Data.gyro_x;
  Angular_Handler.V_Y = IMU_Data.gyro_y;
  Angular_Handler.V_Z = IMU_Data.gyro_z;
  Angular_Handler.Temp = IMU_Data.Temp;
}
//返回Angular_Handle地址
const Angular_Handle *get_Gyro_Angle_Point(void)
{
  return &Angular_Handler;
}
