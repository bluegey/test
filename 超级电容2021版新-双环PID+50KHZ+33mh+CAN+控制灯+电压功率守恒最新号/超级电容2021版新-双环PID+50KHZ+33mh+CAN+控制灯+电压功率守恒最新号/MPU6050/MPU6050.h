#ifndef __MPU6050_H__
#define __MPU6050_H__
#include "sys.h"
#include "IIC.h"
#include "math.h"
#include "timer.h"

#define PI 3.1415927
/*
****************************************
����MPU6050�ڲ���ַ
****************************************
*/
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	MPU6050_CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
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
#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	SLAVE_Address	0xD0	//IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ
/**********************************************
*X Y Z����ٶ�ƫ���������ٶ�ƫ����
**********************************************/
#define ACCEL_X_OFFSET 0.026
#define ACCEL_Y_OFFSET 0.016
#define ACCEL_Z_OFFSET 0.0
#define GYRO_X_OFFSET (-42 / 16.4)
#define GYRO_Y_OFFSET (-2 / 16.4)
#define GYRO_Z_OFFSET -0.012

extern double accel_x, accel_y, accel_z;		//������ļ��ٶ�
extern double accel_pitch, accel_roll, accel_yaw;
extern double gyro_x, gyro_y, gyro_z;			//������Ľ��ٶ�
extern double gyro_pitch, gyro_roll, gyro_yaw;		//ע��accel_yaw��gyro_yaw�ǲ�ͬ��
/*
***********************************************
��������
***********************************************
*/
void MPU6050_Init(void);  					//MPU6050��ʼ��
s32 MPU6050_Get_Data(u8 REG_Address);  		//�ԼĴ����ߵ�8λ���ݺϲ�
void MPU6050_Get_ACCEL(void);				//��ü��ٶȣ���λ��g/ÿ���η��룩
void MPU6050_Get_ACCEL_Angle(void);			//ͨ�����ٶȵõ���̬�ǣ���λ�����ȣ�
void MPU6050_Get_GYRO(void);				//��ý��ٶ�(��λ����/ÿ��)
void MPU6050_Get_GYRO_Angle(void);			//ͨ�����ٶȵõ���̬�ǣ���λ���㣩
#endif
