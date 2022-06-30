#pragma once
#include "sys.h"
#include "stdio.h"
#include "MPU9250.h"
#include "Smooth_Filter.h"
#include "IIR_Filter.h"
#include "Matrix.h"
#include "FreeRTOS.h"
#include "task.h"
#define IMU_Temp_Set 40.0 //陀螺仪预设温度

/*以下宏定义无需修改*/
#ifndef PI
#define PI 3.14159265358979f
#endif
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
#define Mahony_Kp 2.0f // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Mahony_Ki 0.1f // integral gain governs rate of convergence of gyroscope biases

/*以下宏定义视情况修改*/
#define GYROX_DEADLINE 0.002
#define GYROY_DEADLINE 0.002
#define GYROZ_DEADLINE 0.002
#define IMU_Init_Delay_ms(n) delay_ms(n)
#define IMU_Task_Delay_ms(n) vTaskDelay(n)
#define ACCEL_GRAVITY 9.80665f
#define IMU_CALC_INIT_TIME 1000
#define IMU_ENABLE_MAG 0             //是否使用磁力计
#define IMU_ENABLE_ACCEL_CALI_DATA 1 //是否进行加速度校准

typedef struct
{
   float gyro_x;
   float gyro_y;
   float gyro_z;
   float accel_x;
   float accel_y;
   float accel_z;

   float deadline_gyro_x;
   float deadline_gyro_y;
   float deadline_gyro_z;
   float Accel_W_raw_data[6][4];
   float Accel_Scale_data[3][3];
   float Accel_Bias_data[3][1];
   mat Accel_Scale;
   mat Accel_Bias;
#if IMU_ENABLE_MAG
   float mag_x;
   float mag_y;
   float mag_z;
#endif
} IMU_Calibration_Data_t;

typedef struct
{
   IIR_Filter_t IMU_GyroX_Filter;
   IIR_Filter_t IMU_GyroY_Filter;
   IIR_Filter_t IMU_GyroZ_Filter;
   IIR_Filter_t IMU_AccelX_Filter;
   IIR_Filter_t IMU_AccelY_Filter;
   IIR_Filter_t IMU_AccelZ_Filter;
#if IMU_ENABLE_MAG
   SmoothFilter_t IMU_MagX_Filter;
   SmoothFilter_t IMU_MagY_Filter;
   SmoothFilter_t IMU_MagZ_Filter;
#endif
} IMU_Filter_t;

typedef struct
{
   IMU_Calibration_Data_t imu_calibration_data;
   IMU_Filter_t imu_filter;

   float gyro_x;
   float gyro_y;
   float gyro_z;
   float accel_x;
   float accel_y;
   float accel_z;
   float mag_x;
   float mag_y;
   float mag_z;

   double q0;
   double q1;
   double q2;
   double q3;
   float exInt;
   float eyInt;
   float ezInt;

   float pitch;
   float roll;
   float yaw;
   float yaw_last;
   float yaw_new;
   float yaw_count;

   float Temp;
} IMU_Data_t;

class SPI
{
private:
public:
   void MpuSpiInit();
   void SpiSetSpeed(u8 SPI_BaudRatePrescaler);
   unsigned char SPI1_ReadWriteByte(unsigned char TxData);
};
class MPU9250 : public SPI
{
private:
   int Id[2];

public:
   // MPUIO初始化
   bool MPU9250_IoInit();
   // MPU寄存器配置
   bool MPU9250_Init(u8 index);
   //往MPU写入数据
   void MPU9250_Write_Byte(int cs_n, unsigned char reg, unsigned char data);
   //读取MPU数据
   unsigned char MPU9250_Read_Byte(int cs_n, unsigned char reg);
   //获取陀螺仪原始数据
   void IMU_Get_Raw_Data(IMU_Data_t *imu_data);
   //读取校准数据
   void IMU_Flash_Read(float *data, int size, int mode);
   // flash初始化
   void IMU_Init_Flash();
   // MPU时钟初始化
   void IMU_Init_Timer();
   // MPU数据初始化
   void IMU_Data_Init(IMU_Data_t *);
   //角加速度初始化
   void IMU_Init_Calibration_Gyro(IMU_Data_t *imu_data);
   //加速度初始化
   void IMU_Init_Calibration_Accel(IMU_Data_t *imu_data);
   //磁力计初始化
   void IMU_Init_Calibration_Mag(IMU_Data_t *imu_data);
   //滤波初始化
   void IMU_Init_Filter(IMU_Data_t *imu_data);
   //滤波器初始化
   int SmoothFilter_Init(SmoothFilter_t *smooth_filter, int dp);
   //解算初始化
   void IMU_Init_Calc(IMU_Data_t *imu_data);
   //进行最终姿态解算
   void IMU_MahonyAHRSupdate(IMU_Data_t *imu_data);
   //对陀螺仪进行相关计算
   void IMU_Calc(IMU_Data_t *imu_data);
   //时间获取
   float IMU_Get_Time(void);
   //求平方根倒数
   float invSqrt(float x);
   // MPU数据校准
   void IMU_Calibration(IMU_Data_t *imu_data);
   //数据滤波
   void IMU_Filter(IMU_Data_t *imu_data);
   //将原始加速度校准数据进行相关计算
   void IMU_Calibration_Accel(IMU_Data_t *imu_data);
   //将原始角加速度校准数据进行相关计算
   void IMU_Calibration_Gyro(IMU_Data_t *imu_data);
   //将原始磁力计校准数据进行相关计算
   void IMU_Calibration_Mag(IMU_Data_t *imu_data);
   //加速度滤波
   void IMU_Filter_Accel(IMU_Data_t *imu_data);
   //角加速度滤波
   void IMU_Filter_Gyro(IMU_Data_t *imu_data);
   //磁力计滤波
   void IMU_Filter_Mag(IMU_Data_t *imu_data);
   //死区函数
   float IMU_Deadline(float value, float deadline);
   //获取转换后角加速度数据
   void MPU9250_Get_Gyro_transform(float *gx, float *gy, float *gz);
   //将原始加速度数据进行转换为m/^2也就是g   陀螺仪发送的加速度单位为LSB/G
   void MPU9250_Get_Accel_transform(float *ax, float *ay, float *az);
   //获取原始角加速度数据
   void MPU9250_Get_Gyro(short *gx, short *gy, short *gz);
   //获取原始加速度数据
   void MPU9250_Get_Accel(short *ax, short *ay, short *az);
   //读取数据长度
   void MPU9250_Read_Len(int cs_n, unsigned char reg, int len, unsigned char *buf);
   //校准模式
   void IMU_Get_Accel_Cali_Data(IMU_Data_t *imu_data);
   //对获取加速度进行均值滤波处理
   void IMU_Calibration_Accel_Get_Raw(IMU_Data_t *imu_data);
   //计算加速度校准矩阵
   void IMU_Calibration_Accel_Calc_CaliMat(IMU_Data_t *imu_data);
   //写入校准数据
   void IMU_Flash_Write(float *data, int size, int mode);
   //获取陀螺仪温度
   void MPU9250_Get_Temp(float *temp);
   //加权滤波
   float SmoothFilterCalc(SmoothFilter_t *smooth_filter, float data);
   //设置加热占空比
   void MPU9250_Set_Heat_PWM(int duty);
};
