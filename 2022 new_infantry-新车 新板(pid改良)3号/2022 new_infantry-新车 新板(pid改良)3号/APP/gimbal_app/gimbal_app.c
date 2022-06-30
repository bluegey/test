#include "gimbal_app.h"


/*********************************************************************************************************************************************************************/
//使用电机相对角度控制
void gimbal_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_angle_pid, gimbal_motor->relative_angle, gimbal_motor->gimbal_angle_set, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->Error_Kalman);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
//使用电机绝对角度控制电机
void gimbal_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->gimbal_angle_set, gimbal_motor->motor_gyro, gimbal_motor->Error_Kalman);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/*****************************/

void gimbal_pid_reset(Gimbal_PID_t *pid, float p, float i, float d)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    
		pid->err_sum=0;
    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->out  = 0;

}


 void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->err_limit=1.5 ;
		pid->err_dead=0;
		
    pid->err[0] = pid->err[1] = pid->err[2] = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

 fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, extKalman_t Kal_Gim)
{
    fp32 err;
//	  fp32 err_limit=2.0f;
	  fp32 time_change=0.003;//后期测试 采样时间
//    fp32 handle_err;

    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->get = get;
    pid->set = set;
    
		pid->err[2] = pid->err[1];
		pid->err[1] = pid->err[0];
		/***************/
    err = set - get;
    err = KalmanFilter(&Kal_Gim, err);//卡尔曼处理角度误差
    pid->err[0] = /*handle_err*/err;
		
		/***************/
		
		if(pid->pid_mode == NORMAL)
		{
    pid->Pout = pid->kp * pid->err[0];
    pid->Iout += pid->ki * pid->err[0];
    pid->Dout = pid->kd * error_delta;
		}
		else if(pid->pid_mode == I_OUT)
		{
			if(fabs(pid->err[0])>=pid->err_limit)
			{
				pid->Iout =0;
			}
			else
			{
				pid->Iout += pid->ki * pid->err[0];
			}
		pid->Pout = pid->kp * pid->err[0];
    pid->Dout = pid->kd * error_delta;
		}
		else if(pid->pid_mode == RAPEZOID)
		{
		 pid->Pout = pid->kp * pid->err[0];
    pid->Iout += pid->ki * ((pid->err[0]+pid->err[1])/2);
    pid->Dout = pid->kd * (pid->err[0]-pid->err[1]);
		}
		else if(pid->pid_mode == TIME_PID)
		{
		pid->err_sum += pid->err[0]*time_change;
		pid->Pout = pid->kp * pid->err[0];
    pid->Iout = pid->ki * pid->err_sum;
    pid->Dout = pid->kd * (pid->err[0]-pid->err[1])/time_change;
		}
		else
		{
		pid->Pout = pid->kp * pid->err[0];
    pid->Iout += pid->ki * pid->err[0];
    pid->Dout = pid->kd * error_delta;
		}
		
		//死区
		if(fabs(pid->err[0])<=pid->err_dead)
		{
			pid->err[0]=0;
		}
		
		
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

//void GIMBAL_PIDpro_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, extKalman_t Gim)
//{
//	bool error_flag=false;
//}


//得到电机相对角度
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;

    if (center_offset >= 4096)
    {
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else
    {
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }

    return tmp;
}

