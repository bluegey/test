#include "IMU.h"

// SPI初始化
void SPI::MpuSpiInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	SPI_Cmd(SPI1, DISABLE);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	//	SPI1->CR2|=0x20;

	SPI_Cmd(SPI1, ENABLE);

	SPI1_ReadWriteByte(0xF3);
}
// SPI读数据
unsigned char SPI::SPI1_ReadWriteByte(unsigned char TxData)
{
	int Isovertime = 0;
	//等待数据从发送缓冲区传输到移位寄存器
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		Isovertime++;
	}
	//发送移位寄存器数据
	SPI_I2S_SendData(SPI1, TxData);
	//等待数据从移位寄存器拷贝到缓冲区
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{
		Isovertime++;
		if (Isovertime > 200)
		{
			Isovertime = 0;
			SPI_I2S_SendData(SPI1, TxData);
		}
	}
	//读取缓冲区数据同时将RXNE位清零
	return SPI_I2S_ReceiveData(SPI1);
}
//设置SPI传输速度
void SPI::SpiSetSpeed(u8 SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI1->CR1 &= 0XFFC7;
	SPI1->CR1 |= SPI_BaudRatePrescaler;
	SPI_Cmd(SPI1, ENABLE);
}
// MPU9250IO初始化
bool MPU9250::MPU9250_IoInit()
{
	{ //使能引脚初始化
		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(MPU1_CS_GPIO_CLK, ENABLE);
		RCC_AHB1PeriphClockCmd(MPU2_CS_GPIO_CLK, ENABLE);

		GPIO_InitStructure.GPIO_Pin = MPU1_CS_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(MPU1_CS_GPIO_PORT, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = MPU2_CS_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(MPU2_CS_GPIO_PORT, &GPIO_InitStructure);

		GPIO_SetBits(MPU1_CS_GPIO_PORT, MPU1_CS_GPIO_PIN);
		GPIO_SetBits(MPU2_CS_GPIO_PORT, MPU2_CS_GPIO_PIN);
	}
	{ // SPI初始化
		MpuSpiInit();
		GPIO_SetBits(MPU1_CS_GPIO_PORT, MPU1_CS_GPIO_PIN);
		GPIO_SetBits(MPU2_CS_GPIO_PORT, MPU2_CS_GPIO_PIN);
	}
	{ //加热引脚初始化//加热PWM初始化
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_OCInitTypeDef TIM_OCInitStructure;

		RCC_AHB1PeriphClockCmd(MPU_HEAT_GPIO_CLK, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

		GPIO_PinAFConfig(MPU_HEAT_GPIO_PORT, MPU_HEAT_AF_PINSOURCE, GPIO_AF_TIM3);

		GPIO_InitStructure.GPIO_Pin = MPU_HEAT_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(MPU_HEAT_GPIO_PORT, &GPIO_InitStructure);

		GPIO_ResetBits(MPU_HEAT_GPIO_PORT, MPU_HEAT_GPIO_PIN);

		TIM_TimeBaseStructure.TIM_Prescaler = MPU_HEAT_TIM_PSC;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period = MPU_HEAT_TIM_ARR;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC3Init(TIM3, &TIM_OCInitStructure);

		TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

		TIM_ARRPreloadConfig(TIM3, ENABLE);

		TIM_Cmd(TIM3, ENABLE);

		TIM_SetCompare3(TIM3, 0);
	}
	{
		if (MPU9250_Init(1))
			return 1;
		if (MPU9250_Init(2))
			return 1;
	}
	return 0;
}
// MPU9250相关寄存器初始化
bool MPU9250::MPU9250_Init(u8 index)
{
	Id[index - 1] = MPU9250_Read_Byte(1, MPU_DEVICE_ID_REG);
	MPU_delay_ms(10);
	if (Id[index - 1] == 0x70)
	{
		MPU9250_Write_Byte(index, MPU_PWR_MGMT1_REG, 0x80);
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_PWR_MGMT1_REG, 0x00);
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_SIGPATH_RST_REG, 0x07);
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_PWR_MGMT1_REG, 0x03);
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_PWR_MGMT2_REG, 0x00);
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_CFG_REG, 0x00);
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_SAMPLE_RATE_REG, 0x00); // 1K采样率
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_CFG_REG, 0x00); //内部低通滤波频率
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_GYRO_CFG_REG, 3 << 3); //陀螺仪量程，0~3对应250、500、1k、2k（dps）
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_ACCEL_CFG_REG, 0 << 3); //加速度量程，0~3对应2g、4g、8g、16g
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_INT_EN_REG, 0x01); //中断
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_INTBP_CFG_REG, 0x01);
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, 0x27, 0x0D);
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, 0x67, 0x0D);
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, 0x27, 0x0D);
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_PWR_MGMT1_REG, 0x01);
		MPU_delay_ms(10);
		MPU9250_Write_Byte(index, MPU_PWR_MGMT2_REG, 0x00);
		MPU_delay_ms(10);
		return 0;
	}
	else
		return 1;
}
// MPU写入数据
void MPU9250::MPU9250_Write_Byte(int cs_n, unsigned char reg, unsigned char data)
{
	if (cs_n == 1)
		GPIO_ResetBits(MPU1_CS_GPIO_PORT, MPU1_CS_GPIO_PIN);
	else if (cs_n == 2)
		GPIO_ResetBits(MPU2_CS_GPIO_PORT, MPU2_CS_GPIO_PIN);
	SPI1_ReadWriteByte(reg);
	SPI1_ReadWriteByte(data);
	if (cs_n == 1)
		GPIO_SetBits(MPU1_CS_GPIO_PORT, MPU1_CS_GPIO_PIN);
	else if (cs_n == 2)
		GPIO_SetBits(MPU2_CS_GPIO_PORT, MPU2_CS_GPIO_PIN);
}
//读取MPU数据
unsigned char MPU9250::MPU9250_Read_Byte(int cs_n, unsigned char reg)
{
	unsigned char data;

	if (cs_n == 1)
		GPIO_ResetBits(MPU1_CS_GPIO_PORT, MPU1_CS_GPIO_PIN);
	else if (cs_n == 2)
		GPIO_ResetBits(MPU2_CS_GPIO_PORT, MPU2_CS_GPIO_PIN);
	SPI1_ReadWriteByte(reg | 0x80);
	data = SPI1_ReadWriteByte(0xFF);
	if (cs_n == 1)
		GPIO_SetBits(MPU1_CS_GPIO_PORT, MPU1_CS_GPIO_PIN);
	else if (cs_n == 2)
		GPIO_SetBits(MPU2_CS_GPIO_PORT, MPU2_CS_GPIO_PIN);

	return data;
}
// //陀螺仪工作参数初始化
void MPU9250::IMU_Data_Init(IMU_Data_t *imu_data)
{
	imu_data->q0 = 1;
	imu_data->q1 = 0;
	imu_data->q2 = 0;
	imu_data->q3 = 0;
	imu_data->yaw_count = 0;
	imu_data->yaw_last = 0;

	IMU_Init_Timer();
	IMU_Init_Flash();

	IMU_Init_Calibration_Gyro(imu_data);
	IMU_Init_Calibration_Accel(imu_data);
	IMU_Init_Calibration_Mag(imu_data);

	IMU_Init_Filter(imu_data);

	IMU_Init_Calc(imu_data);
}
//时钟初始化
void MPU9250::IMU_Init_Timer()
{
	TIM_TimeBaseInitTypeDef timer_initstruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	timer_initstruct.TIM_CounterMode = TIM_CounterMode_Up;
	timer_initstruct.TIM_Period = 499999999;
	timer_initstruct.TIM_Prescaler = 84 - 1;

	TIM_TimeBaseInit(TIM7, &timer_initstruct);

	TIM_Cmd(TIM7, ENABLE);

	TIM_SetCounter(TIM7, 0);
}
// flash初始化
void MPU9250::IMU_Init_Flash()
{

	return;
}
//角加速度初始化
void MPU9250::IMU_Init_Calibration_Gyro(IMU_Data_t *imu_data)
{

	int i;
	imu_data->imu_calibration_data.gyro_x = 0;
	imu_data->imu_calibration_data.gyro_y = 0;
	imu_data->imu_calibration_data.gyro_z = 0;
	imu_data->imu_calibration_data.deadline_gyro_x = GYROX_DEADLINE;
	imu_data->imu_calibration_data.deadline_gyro_y = GYROY_DEADLINE;
	imu_data->imu_calibration_data.deadline_gyro_z = GYROZ_DEADLINE;
	for (i = 0; i < 200; i++)
	{
		IMU_Get_Raw_Data(imu_data);
		imu_data->imu_calibration_data.gyro_x += imu_data->gyro_x;
		imu_data->imu_calibration_data.gyro_y += imu_data->gyro_y;
		imu_data->imu_calibration_data.gyro_z += imu_data->gyro_z;
		IMU_Init_Delay_ms(3);
	}
	imu_data->imu_calibration_data.gyro_x /= i;
	imu_data->imu_calibration_data.gyro_y /= i;
	imu_data->imu_calibration_data.gyro_z /= i;
}
//加速度初始化
void MPU9250::IMU_Init_Calibration_Accel(IMU_Data_t *imu_data)
{
	int i, j;
	float flash_read_data[4][3] = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1,
		0, 0, 0};
#if IMU_ENABLE_ACCEL_CALI_DATA
	IMU_Flash_Read((float *)flash_read_data, 12, 0);
#endif
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			imu_data->imu_calibration_data.Accel_Scale_data[i][j] = flash_read_data[i][j];
	for (i = 0; i < 3; i++)
		imu_data->imu_calibration_data.Accel_Bias_data[i][0] = flash_read_data[3][i];
	mat_init(&imu_data->imu_calibration_data.Accel_Scale, 3, 3, (float *)imu_data->imu_calibration_data.Accel_Scale_data);
	mat_init(&imu_data->imu_calibration_data.Accel_Bias, 3, 3, (float *)imu_data->imu_calibration_data.Accel_Bias_data);
}
//磁力计初始化
void MPU9250::IMU_Init_Calibration_Mag(IMU_Data_t *imu_data)
{
	return;
}
//滤波初始化
void MPU9250::IMU_Init_Filter(IMU_Data_t *imu_data)
{
	IIR_Filter_Init(&imu_data->imu_filter.IMU_GyroX_Filter);
	IIR_Filter_Init(&imu_data->imu_filter.IMU_GyroY_Filter);
	IIR_Filter_Init(&imu_data->imu_filter.IMU_GyroZ_Filter);
	IIR_Filter_Init(&imu_data->imu_filter.IMU_AccelX_Filter);
	IIR_Filter_Init(&imu_data->imu_filter.IMU_AccelY_Filter);
	IIR_Filter_Init(&imu_data->imu_filter.IMU_AccelZ_Filter);
#if IMU_ENABLE_MAG
	SmoothFilter_Init(&imu_data->imu_filter.IMU_MagX_Filter, 8);
	SmoothFilter_Init(&imu_data->imu_filter.IMU_MagY_Filter, 8);
	SmoothFilter_Init(&imu_data->imu_filter.IMU_MagZ_Filter, 8);
#endif
}
//参数初始化
void MPU9250::IMU_Init_Calc(IMU_Data_t *imu_data)
{
	int i;
	for (i = 0; i < 10; i++)
	{
		IMU_Get_Raw_Data(imu_data);
		IMU_MahonyAHRSupdate(imu_data);
		IMU_Init_Delay_ms(3);
	}
	for (i = 0; i < IMU_CALC_INIT_TIME; i++)
	{
		IMU_Calc(imu_data);
		IMU_Init_Delay_ms(1);
	}
}
//获取原始数据
void MPU9250::IMU_Get_Raw_Data(IMU_Data_t *imu_data)
{

	MPU9250_Get_Gyro_transform(&imu_data->gyro_x, &imu_data->gyro_y, &imu_data->gyro_z);
	MPU9250_Get_Accel_transform(&imu_data->accel_x, &imu_data->accel_y, &imu_data->accel_z);

	/*角加速度转弧度制*/
	imu_data->gyro_x *= DEG2RAD;
	imu_data->gyro_y *= DEG2RAD;
	imu_data->gyro_z *= DEG2RAD;

	imu_data->accel_x *= ACCEL_GRAVITY;
	imu_data->accel_y *= ACCEL_GRAVITY;
	imu_data->accel_z *= ACCEL_GRAVITY;

	imu_data->mag_x = 0;
	imu_data->mag_y = 0;
	imu_data->mag_z = 0;
}
//读取校准数据
void MPU9250::IMU_Flash_Read(float *data, int size, int mode)
{
	int i;
	u32 read_addr = 0x080E0000;
	if (!mode)
		for (i = 0; i < size; i++)
		{
			data[i] = *(float *)read_addr;
			read_addr += 4;
		}
}
//进行最终姿态解算
void MPU9250::IMU_MahonyAHRSupdate(IMU_Data_t *imu_data)
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz;
	float wx, wy, wz;
	float ex, ey, ez;
	float temp0, temp1, temp2, temp3;
	double halfT;

	float q0q0 = imu_data->q0 * imu_data->q0;
	float q0q1 = imu_data->q0 * imu_data->q1;
	float q0q2 = imu_data->q0 * imu_data->q2;
	float q0q3 = imu_data->q0 * imu_data->q3;
	float q1q1 = imu_data->q1 * imu_data->q1;
	float q1q2 = imu_data->q1 * imu_data->q2;
	float q1q3 = imu_data->q1 * imu_data->q3;
	float q2q2 = imu_data->q2 * imu_data->q2;
	float q2q3 = imu_data->q2 * imu_data->q3;
	float q3q3 = imu_data->q3 * imu_data->q3;

	halfT = IMU_Get_Time() / 2000;
	//求平方根倒数
	norm = invSqrt(imu_data->accel_x * imu_data->accel_x + imu_data->accel_y * imu_data->accel_y + imu_data->accel_z * imu_data->accel_z);
	//对每个轴的加速度进行归一化   因为姿态变化矩阵中四元数采用的是规范四元数  两者需对应
	imu_data->accel_x = imu_data->accel_x * norm;
	imu_data->accel_y = imu_data->accel_y * norm;
	imu_data->accel_z = imu_data->accel_z * norm;

#if IMU_ENABLE_MAG
	norm = invSqrt(imu_data->mag_x * imu_data->mag_x + imu_data->mag_y * imu_data->mag_y + imu_data->mag_z * imu_data->mag_z);
	imu_data->mag_x = imu_data->mag_x * norm;
	imu_data->mag_y = imu_data->mag_y * norm;
	imu_data->mag_z = imu_data->mag_z * norm;
#else
	imu_data->mag_x = 0;
	imu_data->mag_y = 0;
	imu_data->mag_z = 0;
#endif

	// compute reference direction of flux
	hx = 2.0f * imu_data->mag_x * (0.5f - q2q2 - q3q3) + 2.0f * imu_data->mag_y * (q1q2 - q0q3) + 2.0f * imu_data->mag_z * (q1q3 + q0q2);
	hy = 2.0f * imu_data->mag_x * (q1q2 + q0q3) + 2.0f * imu_data->mag_y * (0.5f - q1q1 - q3q3) + 2.0f * imu_data->mag_z * (q2q3 - q0q1);
	hz = 2.0f * imu_data->mag_x * (q1q3 - q0q2) + 2.0f * imu_data->mag_y * (q2q3 + q0q1) + 2.0f * imu_data->mag_z * (0.5f - q1q1 - q2q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;
	// estimated direction of gravity and flux (v and w)
	//提前物体坐标系下的重力分量
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3; // 1-2*q1*q1-2*q2*q2
	//计算磁力盘数据
	wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
	wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
	wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	//计算姿态误差
	ex = (imu_data->accel_y * vz - imu_data->accel_z * vy) + (imu_data->mag_y * wz - imu_data->mag_z * wy);
	ey = (imu_data->accel_z * vx - imu_data->accel_x * vz) + (imu_data->mag_z * wx - imu_data->mag_x * wz);
	ez = (imu_data->accel_x * vy - imu_data->accel_y * vx) + (imu_data->mag_x * wy - imu_data->mag_y * wx);
	//
	if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		//对误差积分
		imu_data->exInt = imu_data->exInt + ex * Mahony_Ki * halfT;
		imu_data->eyInt = imu_data->eyInt + ey * Mahony_Ki * halfT;
		imu_data->ezInt = imu_data->ezInt + ez * Mahony_Ki * halfT;
		//进行互补滤波 修正角速度值
		imu_data->gyro_x = imu_data->gyro_x + Mahony_Kp * ex + imu_data->exInt;
		imu_data->gyro_y = imu_data->gyro_y + Mahony_Kp * ey + imu_data->eyInt;
		imu_data->gyro_z = imu_data->gyro_z + Mahony_Kp * ez + imu_data->ezInt;
	}

	// integrate quaternion rate and normalise
	//解四元数微分方程
	temp0 = imu_data->q0 + (-imu_data->q1 * imu_data->gyro_x - imu_data->q2 * imu_data->gyro_y - imu_data->q3 * imu_data->gyro_z) * halfT;
	temp1 = imu_data->q1 + (imu_data->q0 * imu_data->gyro_x + imu_data->q2 * imu_data->gyro_z - imu_data->q3 * imu_data->gyro_y) * halfT;
	temp2 = imu_data->q2 + (imu_data->q0 * imu_data->gyro_y - imu_data->q1 * imu_data->gyro_z + imu_data->q3 * imu_data->gyro_x) * halfT;
	temp3 = imu_data->q3 + (imu_data->q0 * imu_data->gyro_z + imu_data->q1 * imu_data->gyro_y - imu_data->q2 * imu_data->gyro_x) * halfT;

	// normalise quaternion
	//对四元数进行归一化处理
	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	imu_data->q0 = temp0 * norm;
	imu_data->q1 = temp1 * norm;
	imu_data->q2 = temp2 * norm;
	imu_data->q3 = temp3 * norm;
	//计算姿态解算角度
	imu_data->yaw_last = imu_data->yaw_new;
	imu_data->yaw_new = -atan2(2.0f * imu_data->q1 * imu_data->q2 + 2.0f * imu_data->q0 * imu_data->q3, -2.0f * imu_data->q2 * imu_data->q2 - 2.0f * imu_data->q3 * imu_data->q3 + 1.0f) * RAD2DEG;
	imu_data->pitch = -asin(-2.0f * imu_data->q1 * imu_data->q3 + 2.0f * imu_data->q0 * imu_data->q2) * RAD2DEG;
	imu_data->roll = atan2(2.0f * imu_data->q2 * imu_data->q3 + 2 * imu_data->q0 * imu_data->q1, -2.0f * imu_data->q1 * imu_data->q1 - 2.0f * imu_data->q2 * imu_data->q2 + 1.0f) * RAD2DEG;
}
//对陀螺仪进行相关计算
void MPU9250::IMU_Calc(IMU_Data_t *imu_data)
{
	//读取陀螺仪数据
	IMU_Get_Raw_Data(imu_data);
	//将原始数据与校准值进行相关计算
	IMU_Calibration(imu_data); //
	//对获取数据进行滤波处理
	IMU_Filter(imu_data);
	//进行最终姿态解算
	IMU_MahonyAHRSupdate(imu_data);
	if ((imu_data->yaw_new - imu_data->yaw_last) >= 330)
	{
		imu_data->yaw_count--;
	}
	else if ((imu_data->yaw_new - imu_data->yaw_last) <= -330)
	{
		imu_data->yaw_count++;
	}
	imu_data->yaw = imu_data->yaw_count * 360 + imu_data->yaw_new;
}
//陀螺仪校准
void MPU9250::IMU_Calibration(IMU_Data_t *imu_data)
{
	//将原始加速度校准数据进行相关计算
	IMU_Calibration_Accel(imu_data);
	//将原始角加速度校准数据进行相关计算
	IMU_Calibration_Gyro(imu_data);
	//将原始角磁力计校准数据进行相关计算
	IMU_Calibration_Mag(imu_data);
}
//获取系统时间
float MPU9250::IMU_Get_Time(void)
{
	float new_time;
	new_time = TIM7->CNT;
	TIM_SetCounter(TIM7, 0);
	return (new_time / 1000);
}
//开方函数
float MPU9250::invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;

	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}
//数据滤波
void MPU9250::IMU_Filter(IMU_Data_t *imu_data)
{
	IMU_Filter_Accel(imu_data);
	IMU_Filter_Gyro(imu_data);
	IMU_Filter_Mag(imu_data);
}
//将原始加速度校准数据进行相关计算
void MPU9250::IMU_Calibration_Accel(IMU_Data_t *imu_data)
{
	float accel_raw_data[3][1];
	float temp3x1_1_data[3][1], temp3x1_2_data[3][1];
	mat accel_raw;
	mat temp3x1_1, temp3x1_2;
	mat_init(&accel_raw, 3, 1, (float *)accel_raw_data);
	mat_init(&temp3x1_1, 3, 1, (float *)temp3x1_1_data);
	mat_init(&temp3x1_2, 3, 1, (float *)temp3x1_2_data);
	accel_raw.pData[0] = imu_data->accel_x;
	accel_raw.pData[1] = imu_data->accel_y;
	accel_raw.pData[2] = imu_data->accel_z;
	mat_mult(&imu_data->imu_calibration_data.Accel_Scale, &accel_raw, &temp3x1_1);
	accel_raw.pData[0] = imu_data->imu_calibration_data.Accel_Bias_data[0][0];
	accel_raw.pData[1] = imu_data->imu_calibration_data.Accel_Bias_data[1][0];
	accel_raw.pData[2] = imu_data->imu_calibration_data.Accel_Bias_data[2][0];
	mat_add(&temp3x1_1, &accel_raw, &temp3x1_2);
	imu_data->accel_x = temp3x1_2.pData[0];
	imu_data->accel_y = temp3x1_2.pData[1];
	imu_data->accel_z = temp3x1_2.pData[2];
}
////将原始角加速度校准数据进行相关计算
void MPU9250::IMU_Calibration_Gyro(IMU_Data_t *imu_data)
{
	imu_data->gyro_x -= imu_data->imu_calibration_data.gyro_x;
	imu_data->gyro_y -= imu_data->imu_calibration_data.gyro_y;
	imu_data->gyro_z -= imu_data->imu_calibration_data.gyro_z;
}
////将原始磁力计校准数据进行相关计算
void MPU9250::IMU_Calibration_Mag(IMU_Data_t *imu_data)
{
	return;
}
//加速度滤波
void MPU9250::IMU_Filter_Accel(IMU_Data_t *imu_data)
{
	imu_data->accel_x = IIR_Filter_Calc(&imu_data->imu_filter.IMU_AccelX_Filter, imu_data->accel_x);
	imu_data->accel_y = IIR_Filter_Calc(&imu_data->imu_filter.IMU_AccelY_Filter, imu_data->accel_y);
	imu_data->accel_z = IIR_Filter_Calc(&imu_data->imu_filter.IMU_AccelZ_Filter, imu_data->accel_z);
}
//加速度滤波
void MPU9250::IMU_Filter_Gyro(IMU_Data_t *imu_data)
{
	imu_data->gyro_x = IIR_Filter_Calc(&imu_data->imu_filter.IMU_GyroX_Filter, imu_data->gyro_x);
	imu_data->gyro_y = IIR_Filter_Calc(&imu_data->imu_filter.IMU_GyroY_Filter, imu_data->gyro_y);
	imu_data->gyro_z = IIR_Filter_Calc(&imu_data->imu_filter.IMU_GyroZ_Filter, imu_data->gyro_z);
	imu_data->gyro_x = IMU_Deadline(imu_data->gyro_x, imu_data->imu_calibration_data.deadline_gyro_x);
	imu_data->gyro_y = IMU_Deadline(imu_data->gyro_y, imu_data->imu_calibration_data.deadline_gyro_y);
	imu_data->gyro_z = IMU_Deadline(imu_data->gyro_z, imu_data->imu_calibration_data.deadline_gyro_z);
}
//磁力计滤波
void MPU9250::IMU_Filter_Mag(IMU_Data_t *imu_data)
{
#if IMU_ENABLE_MAG
	imu_data->mag_x = SmoothFilter_Calc(&imu_data->imu_filter.IMU_MagX_Filter, imu_data->mag_x);
	imu_data->mag_y = SmoothFilter_Calc(&imu_data->imu_filter.IMU_MagY_Filter, imu_data->mag_y);
	imu_data->mag_z = SmoothFilter_Calc(&imu_data->imu_filter.IMU_MagZ_Filter, imu_data->mag_z);
#endif
}
//死区控制函数
float MPU9250::IMU_Deadline(float value, float deadline)
{
	if (fabs(value) < deadline)
		return 0;
	return value;
}
//将原始角加速度转化为°/s，  陀螺仪发送角加速度单位为LSB/（°/s）
void MPU9250::MPU9250_Get_Gyro_transform(float *gx, float *gy, float *gz)
{
	short rgx, rgy, rgz;

	MPU9250_Get_Gyro(&rgx, &rgy, &rgz);
	// 2000dps量程   分辨率为16.4  其倒数约为0.060976f
	*gx = (rgx * 0.060976f);
	*gy = (rgy * 0.060976f);
	*gz = (rgz * 0.060976f);
}
////将原始加速度数据进行转换为m/^2也就是g   陀螺仪发送的加速度单位为LSB/G
void MPU9250::MPU9250_Get_Accel_transform(float *ax, float *ay, float *az)
{
	short rax, ray, raz;
	//读取原始数据
	MPU9250_Get_Accel(&rax, &ray, &raz);
	//  进行数据转换  1g=16384
	*ax = (float)rax / 16384;
	*ay = (float)ray / 16384;
	*az = (float)raz / 16384;
}
//获取原始角加速度数据
void MPU9250::MPU9250_Get_Gyro(short *gx, short *gy, short *gz)
{
	unsigned char buf[6];
	short raw1[3], raw2[3];

	MPU9250_Read_Len(1, MPU_GYRO_XOUTH_REG, 6, buf);
	raw1[0] = ((unsigned short)buf[0] << 8) | buf[1];
	raw1[1] = ((unsigned short)buf[2] << 8) | buf[3];
	raw1[2] = ((unsigned short)buf[4] << 8) | buf[5];
	MPU9250_Read_Len(2, MPU_GYRO_XOUTH_REG, 6, buf);
	raw2[0] = ((unsigned short)buf[0] << 8) | buf[1];
	raw2[1] = ((unsigned short)buf[2] << 8) | buf[3];
	raw2[2] = ((unsigned short)buf[4] << 8) | buf[5];

	*gx = (raw1[0] + raw2[1]) / 2;
	*gy = (raw1[1] - raw2[0]) / 2;
	*gz = (raw1[2] + raw2[2]) / 2;
}
//获取原始加速度数据
void MPU9250::MPU9250_Get_Accel(short *ax, short *ay, short *az)
{

	unsigned char Buffer[6] = {0};
	short raw1[3], raw2[3];
	for (int i = 0; i < 6; i++)
	{
		Buffer[i] = 0;
	}
	MPU9250_Read_Len(1, MPU_ACCEL_XOUTH_REG, 6, Buffer);
	raw1[0] = ((unsigned short)Buffer[0] << 8) | Buffer[1];
	raw1[1] = ((unsigned short)Buffer[2] << 8) | Buffer[3];
	raw1[2] = ((unsigned short)Buffer[4] << 8) | Buffer[5];
	MPU9250_Read_Len(2, MPU_ACCEL_XOUTH_REG, 6, Buffer);
	raw2[0] = ((unsigned short)Buffer[0] << 8) | Buffer[1];
	raw2[1] = ((unsigned short)Buffer[2] << 8) | Buffer[3];
	raw2[2] = ((unsigned short)Buffer[4] << 8) | Buffer[5];

	*ax = (raw1[0] + raw2[1]) / 2;
	*ay = (raw1[1] - raw2[0]) / 2;
	*az = (raw1[2] + raw2[2]) / 2;
}
//读取数据长度
void MPU9250::MPU9250_Read_Len(int cs_n, unsigned char reg, int len, unsigned char *buf)
{
	if (cs_n == 1)
		GPIO_ResetBits(MPU1_CS_GPIO_PORT, MPU1_CS_GPIO_PIN);
	else if (cs_n == 2)
		GPIO_ResetBits(MPU2_CS_GPIO_PORT, MPU2_CS_GPIO_PIN);
	SPI1_ReadWriteByte(reg | 0x80);
	while (len--)
	{
		*(buf++) = SPI1_ReadWriteByte(0xFF);
		reg++;
	}
	if (cs_n == 1)
		GPIO_SetBits(MPU1_CS_GPIO_PORT, MPU1_CS_GPIO_PIN);
	else if (cs_n == 2)
		GPIO_SetBits(MPU2_CS_GPIO_PORT, MPU2_CS_GPIO_PIN);
}
//滤波器初始化
int MPU9250::SmoothFilter_Init(SmoothFilter_t *smooth_filter, int dp)
{

	int i;
#if SMOOTH_FILTER_DATA_DP > 8
	if (dp > SMOOTH_FILTER_DATA_DP)
		return 1;
#else
	if (dp > 8)
		return 1;
#endif
	smooth_filter->dp = dp;
	smooth_filter->pointer = 0;
	for (i = 0; i < 4; i++)
	{
		smooth_filter->raw_data[i] = 0;
	}
	return 0;
}
//六面加速度校准
void MPU9250::IMU_Get_Accel_Cali_Data(IMU_Data_t *imu_data)
{
	static int IsCalibration = 0, Pos_N = 0;
	while (!IsCalibration)
	{
		if ((Pos_N > 0) && (Pos_N <= 6))
		{
			//加速度测量值  最后一行为零点漂移
			IMU_Calibration_Accel_Get_Raw(imu_data); //获取陀螺仪数据
			imu_data->imu_calibration_data.Accel_W_raw_data[Pos_N - 1][0] = imu_data->imu_calibration_data.accel_x;
			imu_data->imu_calibration_data.Accel_W_raw_data[Pos_N - 1][1] = imu_data->imu_calibration_data.accel_y;
			imu_data->imu_calibration_data.Accel_W_raw_data[Pos_N - 1][2] = imu_data->imu_calibration_data.accel_z;
			imu_data->imu_calibration_data.Accel_W_raw_data[Pos_N - 1][3] = 1;
			Pos_N = 0;
		}
		else if (Pos_N == 7)
		{
			IMU_Calibration_Accel_Calc_CaliMat(imu_data);
			Pos_N = 0;
		}
		IMU_Task_Delay_ms(1);
	}
}
//对获取加速度进行均值滤波处理
void MPU9250::IMU_Calibration_Accel_Get_Raw(IMU_Data_t *imu_data)
{
	int i;
	for (i = 0; i < 200; i++)
	{
		IMU_Get_Raw_Data(imu_data); //获取陀螺仪数据
		imu_data->imu_calibration_data.accel_x += imu_data->accel_x;
		imu_data->imu_calibration_data.accel_y += imu_data->accel_y;
		imu_data->imu_calibration_data.accel_z += imu_data->accel_z;
		IMU_Task_Delay_ms(3);
	}
	imu_data->imu_calibration_data.accel_x /= i;
	imu_data->imu_calibration_data.accel_y /= i;
	imu_data->imu_calibration_data.accel_z /= i;
}
//计算加速度校准矩阵
void MPU9250::IMU_Calibration_Accel_Calc_CaliMat(IMU_Data_t *imu_data)
{

	float Accel_X_cali_data[4][3];
	//加速度六面单位矩阵
	float Accel_Y_pos_data[6][3] =
		{
			     0,                    0,              ACCEL_GRAVITY,  // z面
			     0,                    0,             -ACCEL_GRAVITY,  // -z面
			     0,               ACCEL_GRAVITY,            0,         // y面
			     0,              -ACCEL_GRAVITY,            0,         //-y面
			 ACCEL_GRAVITY,             0,                  0,         // x面
			-ACCEL_GRAVITY,             0,                  0,         //-x面
		};
	//存放陀螺仪转置矩阵
	float Accel_W_raw_T_data[4][6];
	mat Accel_X_cali;
	//陀螺仪的6面单位矩阵
	mat Accel_Y_pos;
	// Accel_W_raw  存放陀螺仪实际数据  Accel_W_raw_T   存放陀螺仪转置矩阵
	mat Accel_W_raw, Accel_W_raw_T;

	float temp4x4_1_data[4][4], temp4x4_2_data[4][4];
	float temp4x6_data[4][6];
	mat temp4x4_1, temp4x4_2, temp4x6;

	mat_init(&Accel_X_cali, 4, 3, (float *)Accel_X_cali_data);
	mat_init(&Accel_Y_pos, 6, 3, (float *)Accel_Y_pos_data);
	mat_init(&Accel_W_raw, 6, 4, (float *)imu_data->imu_calibration_data.Accel_W_raw_data);
	mat_init(&Accel_W_raw_T, 4, 6, (float *)Accel_W_raw_T_data);
	mat_init(&temp4x4_1, 4, 4, (float *)temp4x4_1_data);
	mat_init(&temp4x4_2, 4, 4, (float *)temp4x4_2_data);
	mat_init(&temp4x6, 4, 6, (float *)temp4x6_data);

	while (mat_trans(&Accel_W_raw, &Accel_W_raw_T)) //
		;
	while (mat_mult(&Accel_W_raw_T, &Accel_W_raw, &temp4x4_1)) //
		;
	while (mat_inv(&temp4x4_1, &temp4x4_2))
		;
	while (mat_mult(&temp4x4_2, &Accel_W_raw_T, &temp4x6))
		;
	while (mat_mult(&temp4x6, &Accel_Y_pos, &Accel_X_cali))
		;
	IMU_Flash_Write((float *)Accel_X_cali.pData, 12, 0);
}
//写入校准后的陀螺仪数据
void MPU9250::IMU_Flash_Write(float *data, int size, int mode)
{
	int i;
	u32 write_addr = 0x080E0000;
	while (!mode)
	{
		FLASH_Unlock();
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
		FLASH_EraseSector(FLASH_Sector_11, VoltageRange_2);

		for (i = 0; i < size; i++)
		{
			FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
			FLASH_ProgramWord(write_addr, *((u32 *)&data[i]));
			write_addr += 4;
		}
		FLASH_Lock();
		break;
	}
}
//获取陀螺仪温度
void MPU9250::MPU9250_Get_Temp(float *temp)
{
	unsigned char buf[2];
	short raw1, raw2;

	MPU9250_Read_Len(1, MPU_TEMP_OUTH_REG, 2, buf);
	raw1 = ((short)(buf[0]) << 8) | buf[1];
	MPU9250_Read_Len(2, MPU_TEMP_OUTH_REG, 2, buf);
	raw2 = ((short)(buf[0]) << 8) | buf[1];
	*temp = (((double)raw1 / 333.87 + 21) + ((double)raw2 / 333.87 + 21)) / 2;
}
//滤波
float MPU9250::SmoothFilterCalc(SmoothFilter_t *smooth_filter, float data)
{

	int i;
	float ave = 0;
	smooth_filter->raw_data[smooth_filter->pointer] = data;
	if (++smooth_filter->pointer == smooth_filter->dp)
		smooth_filter->pointer = 0;
	for (i = 0; i < smooth_filter->dp; i++)
	{
		ave += smooth_filter->raw_data[i];
	}
	ave /= smooth_filter->dp;
	return ave;
}
//设置加热占空比
void MPU9250::MPU9250_Set_Heat_PWM(int duty)
{
	//注意限幅，避免PID输出负数或超定时器计数值
	if (duty > 1000)
		duty = 1000;
	if (duty < 0)
		duty = 0;
	TIM_SetCompare3(TIM3, duty);
}
