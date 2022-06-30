#include "MPU6050.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
//变量定义
//////////////////////////////////////////////////////////////////////////////////////////////////
/*************************************************************************
*加速度与角速度
*************************************************************************/
double accel_x, accel_y, accel_z;		//三个轴的加速度
double accel_pitch, accel_roll, accel_yaw;
double gyro_x, gyro_y, gyro_z;			//三个轴的角速度
double gyro_pitch, gyro_roll, gyro_yaw;		//注意accel_yaw与gyro_yaw是不同的
//////////////////////////////////////////////////////////////////////////////////////////////////
//										函数定义
//////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************
MPU6050初始化
**************************************************************************/
void MPU6050_Init(){
	IIC_GPIO_Init();
	IIC_SingleWrite(SLAVE_Address, PWR_MGMT_1, 0x00);	//解除休眠状态
	IIC_SingleWrite(SLAVE_Address, SMPLRT_DIV, 0x07);
	IIC_SingleWrite(SLAVE_Address, MPU6050_CONFIG, 0x06);
	IIC_SingleWrite(SLAVE_Address, GYRO_CONFIG, 0x18);		 //+-2000度/s
	IIC_SingleWrite(SLAVE_Address, ACCEL_CONFIG, 0x01);		 //+-2g
	TIM3_IRQ_Num = 0;
}
/*
**************************************************************************
MPU6050获取寄存器数据
**************************************************************************
*/

s32 MPU6050_Get_Data(u8 REG_Address){
	s32 Data;
	u8 Data_H, Data_L;
	Data_H = IIC_SingleRead(SLAVE_Address, REG_Address);
	Data_L = IIC_SingleRead(SLAVE_Address, REG_Address + 1);
	Data = (s32)((s16)((Data_H << 8) + Data_L));
	return Data;
}
/*
**************************************************************************
计算加速度
**************************************************************************
*/
void MPU6050_Get_ACCEL(){
	accel_x = (double)MPU6050_Get_Data(ACCEL_XOUT_H) / 16384.00 - ACCEL_X_OFFSET;
	accel_y = (double)MPU6050_Get_Data(ACCEL_YOUT_H) / 16384.00 - ACCEL_Y_OFFSET;
	accel_z = (double)MPU6050_Get_Data(ACCEL_ZOUT_H) / 16384.00 - ACCEL_Z_OFFSET;
}
/*
**************************************************************************
*通过加速度得到姿态角pitch、roll、yaw
**************************************************************************
*/
void MPU6050_Get_ACCEL_Angle(){
	MPU6050_Get_ACCEL();
	accel_pitch = atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * (180 / PI);
//	accel_pitch = atan(100000) * (180 / 3.14);
	accel_roll  = atan(accel_y / sqrt(accel_x * accel_x + accel_z * accel_z)) * (180 / PI);
	accel_yaw   = atan(sqrt(accel_x * accel_x + accel_y * accel_y) / accel_z) * (180 / PI);	
}
/*
**************************************************************************
*计算角速度
**************************************************************************
*/
void MPU6050_Get_GYRO(){
	gyro_x = (double)(MPU6050_Get_Data(GYRO_XOUT_H) / 16.4 - GYRO_X_OFFSET);
	gyro_y = (double)(MPU6050_Get_Data(GYRO_YOUT_H) / 16.4 - GYRO_Y_OFFSET);
	gyro_z = (double)(MPU6050_Get_Data(GYRO_ZOUT_H) / 16.4 - GYRO_Z_OFFSET);
}
/*
**************************************************************************
*通过角速度计算姿态角
**************************************************************************
*/
void MPU6050_Get_GYRO_Angle(){
	MPU6050_Get_GYRO();
	gyro_pitch += (gyro_x * TIM3_IRQ_Num / 1200.00);
	gyro_roll += (gyro_y * TIM3_IRQ_Num / 1200.00);
	gyro_yaw += (gyro_z * TIM3_IRQ_Num / 1200.00);
	TIM3_IRQ_Num = 0;
}
