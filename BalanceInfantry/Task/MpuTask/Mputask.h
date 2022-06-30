#pragma once
#include "IMU.h"
#include "PID.h"

#define IMU_Temp_PID_Kp 300.0
#define IMU_Temp_PID_Ki 0
#define IMU_Temp_PID_Kd 0
#define IMU_Temp_PID_POUT 1000
#define IMU_Temp_PID_IOUT 0
#define IMU_Temp_PID_DOUT 0
#define IMU_Temp_PID_OUT 1000

typedef struct
{
    short V_X;
    short V_Y;
    short V_Z;
    float Pitch;
    float ROLL;
    float YAW;
    float Temp;
} Angular_Handle;

extern MPU9250 MPU9250Str;

extern const Angular_Handle *get_Gyro_Angle_Point(void);
extern void IMU_task(void *pvParameters);
void IMU_Translate();
void Mpu9250Init(void);
// class IMUCTRL
// {
// private:
//     bool WorkMode;


// public:
//     IMUCTRL();
//     void ImuCtrlInit();
// };


