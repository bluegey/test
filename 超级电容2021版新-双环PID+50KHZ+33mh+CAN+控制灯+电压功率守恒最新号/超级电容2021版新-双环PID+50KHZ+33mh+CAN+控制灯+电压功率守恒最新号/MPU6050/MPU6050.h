#ifndef __MPU6050_H__
#define __MPU6050_H__
#include "sys.h"
#include "IIC.h"
#include "math.h"
#include "timer.h"

#define PI 3.1415927
/*
****************************************
定义MPU6050内部地址
****************************************
*/
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	MPU6050_CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	SLAVE_Address	0xD0	//IIC写入时的地址字节数据，+1为读取
/**********************************************
*X Y Z轴加速度偏移量及角速度偏移量
**********************************************/
#define ACCEL_X_OFFSET 0.026
#define ACCEL_Y_OFFSET 0.016
#define ACCEL_Z_OFFSET 0.0
#define GYRO_X_OFFSET (-42 / 16.4)
#define GYRO_Y_OFFSET (-2 / 16.4)
#define GYRO_Z_OFFSET -0.012

extern double accel_x, accel_y, accel_z;		//三个轴的加速度
extern double accel_pitch, accel_roll, accel_yaw;
extern double gyro_x, gyro_y, gyro_z;			//三个轴的角速度
extern double gyro_pitch, gyro_roll, gyro_yaw;		//注意accel_yaw与gyro_yaw是不同的
/*
***********************************************
函数定义
***********************************************
*/
void MPU6050_Init(void);  					//MPU6050初始化
s32 MPU6050_Get_Data(u8 REG_Address);  		//对寄存器高低8位数据合并
void MPU6050_Get_ACCEL(void);				//获得加速度（单位：g/每二次方秒）
void MPU6050_Get_ACCEL_Angle(void);			//通过加速度得到姿态角（单位：弧度）
void MPU6050_Get_GYRO(void);				//获得角速度(单位：°/每秒)
void MPU6050_Get_GYRO_Angle(void);			//通过角速度得到姿态角（单位：°）
#endif
