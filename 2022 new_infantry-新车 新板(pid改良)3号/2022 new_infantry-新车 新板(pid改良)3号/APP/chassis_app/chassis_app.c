#include "chassis_app.h"

chassis_move_t chassis_move_e;
/*********************************************************************��������*****************************************************************************************/
#define WARNING_ENERGY	60
float toatl_speed_err,out_speed_err[4],finaal_out[4];
void Chassis_Power_Limit(chassis_move_t *power_ctrl)
{
    /*********************�洫�㷨*************************/
    float    kLimit = 0.0f;//��������ϵ��
    float 	 fTotalCurrentLimit;
    float    chassis_totaloutput = 0.0f;//ͳ�����������
    float    Joule_Residue = 0.0f;//ʣ�ཹ����������
    static 	 int16_t judgDataError_Time = 0;

    Joule_Residue = power_ctrl->chassis_power_measure->chassis_power_buffer;//ʣ�ཹ������

    //ͳ�Ƶ��������
	for(int i=0;i<4;i++)
	{
	out_speed_err[i]=power_ctrl->wheel_spd_ref[i]-power_ctrl->wheel_spd_fdb[i];
	}
	toatl_speed_err=abs((int16_t)out_speed_err[0])+abs((int16_t)out_speed_err[1])+abs((int16_t)out_speed_err[2])+abs((int16_t)out_speed_err[3]);
    chassis_totaloutput = abs((int16_t)power_ctrl->motor_speed_pid[0].out) + abs((int16_t)power_ctrl->motor_speed_pid[1].out) 
																						+ abs((int16_t)power_ctrl->motor_speed_pid[2].out) + abs((int16_t)power_ctrl->motor_speed_pid[3].out);

    if(power_ctrl->chassis_monitor_point[JudgementWDG].errorExist == 1)//����ϵͳ��Чʱǿ������
    {
        judgDataError_Time++;

        if(judgDataError_Time > 100)
        {
            for (u8 i = 0; i < 4; i++)
            {
                VAL_LIMIT(power_ctrl->motor_speed_pid[i].out, -3000, 3000);
            }
        }
    }

    if(power_ctrl->chassis_monitor_point[JudgementWDG].errorExist == 0)
    {
        judgDataError_Time = 0;

        //ʣ�ཹ������С,��ʼ�������,����ϵ��Ϊƽ����ϵ
        if(Joule_Residue < WARNING_ENERGY)
        {
            kLimit = (float)(Joule_Residue / WARNING_ENERGY)	* (float)(Joule_Residue / WARNING_ENERGY);//ϵ��Ϊ (��ǰʣ�ཹ��/60)��ƽ��
            fTotalCurrentLimit = (kLimit * M3505_MOTOR_SPEED_PID_MAX_OUT * 4);
        }
        else //���������ָ���һ����ֵ
        {
            fTotalCurrentLimit = (M3505_MOTOR_SPEED_PID_MAX_OUT * 4);
        }

        //���̸�����������·���
        if (chassis_totaloutput > fTotalCurrentLimit)
        {
            //��ֵ����ֵ
//            power_ctrl->motor_speed_pid[0].out = ((power_ctrl->motor_speed_pid[0].out) / chassis_totaloutput * fTotalCurrentLimit);
//            power_ctrl->motor_speed_pid[1].out = ((power_ctrl->motor_speed_pid[1].out) / chassis_totaloutput * fTotalCurrentLimit);
//            power_ctrl->motor_speed_pid[2].out = ((power_ctrl->motor_speed_pid[2].out) / chassis_totaloutput * fTotalCurrentLimit);
//            power_ctrl->motor_speed_pid[3].out = ((power_ctrl->motor_speed_pid[3].out) / chassis_totaloutput * fTotalCurrentLimit);
					    power_ctrl->motor_speed_pid[0].out = out_speed_err[0]/toatl_speed_err* fTotalCurrentLimit;
				     	power_ctrl->motor_speed_pid[1].out = out_speed_err[1]/toatl_speed_err* fTotalCurrentLimit;
				    	power_ctrl->motor_speed_pid[2].out = out_speed_err[2]/toatl_speed_err* fTotalCurrentLimit;
				    	power_ctrl->motor_speed_pid[3].out = out_speed_err[3]/toatl_speed_err* fTotalCurrentLimit;
        }
				for(int i=0;i<4;i++)
				{
				finaal_out[i]=power_ctrl->motor_speed_pid[i].out;
				}
    }
}
/*********************************************************************��������**************************************************************************************/
void Super_power_ctrl(chassis_move_t *power_ctrl)
{
    uint16_t fTotalpowerLimit;

    if(power_ctrl->chassis_monitor_point[JudgementWDG].errorExist == 0)
    {
        fTotalpowerLimit = power_ctrl->chassis_status_measure->chassis_power_limit;
        CAN_CMD_SUPERPOWER(fTotalpowerLimit, Shift_Flag,power_ctrl->chassis_power_measure->chassis_power_buffer);//�л���Դ
    }
    else if(power_ctrl->chassis_monitor_point[JudgementWDG].errorExist == 1)
    {
        CAN_CMD_SUPERPOWER(20, Shift_Flag,0);//20W���ʹ��
    }
}
/************************************************************************ȫ���㷨****************************************************************************************/
float   wheel_rpm[4],buffer_time=0,uphill_ratio=0.001f,max_speed=1600.0f,max_limit_angle=10.0f;
void mecanum_calc(float vx, float vy, float vz, float speed[], chassis_move_t *power_ctrl)
{
    static float rotate_ratio_fr;//ǰ��
    static float rotate_ratio_fl;//ǰ��
    static float rotate_ratio_bl;//����
    static float rotate_ratio_br;//����
    static float wheel_rpm_ratio;

    rotate_ratio_fr 	= ((WHEELBASE + WHEELTRACK) / 2.0f - chassis_move_e.rotate_x_offset + chassis_move_e.rotate_y_offset) / RADIAN_COEF; //6.63
    rotate_ratio_fl 	= ((WHEELBASE + WHEELTRACK) / 2.0f - chassis_move_e.rotate_x_offset - chassis_move_e.rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + chassis_move_e.rotate_x_offset - chassis_move_e.rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + chassis_move_e.rotate_x_offset + chassis_move_e.rotate_y_offset) / RADIAN_COEF;

    wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO); //25.157

    uint16_t MAX_WHEEL_RPM;
    float   max = 0;

	
//	            MAX_WHEEL_RPM = 10000 ;

    if(Ctrl_Flag == 1)
    {
        MAX_WHEEL_RPM = 1500;
    }

    else if((Super_power.volt < 12000 || Shift_Flag == 2) && Ctrl_Flag == 0)
    {
        if(power_ctrl->chassis_status_measure->chassis_power_limit == 40 || power_ctrl->chassis_status_measure->chassis_power_limit == 45
                || power_ctrl->chassis_status_measure->chassis_power_limit == 50)
        {
            MAX_WHEEL_RPM = 4300 ;
        }
        else if(power_ctrl->chassis_status_measure->chassis_power_limit == 55)
        {
            MAX_WHEEL_RPM = 4700 ;
        }
        else if(power_ctrl->chassis_status_measure->chassis_power_limit == 60)
        {
            MAX_WHEEL_RPM = 5000 ;
        }
        else if(power_ctrl->chassis_status_measure->chassis_power_limit == 80)
        {
            MAX_WHEEL_RPM = 6000 ;
        }
        else if(power_ctrl->chassis_status_measure->chassis_power_limit == 100)
        {
            MAX_WHEEL_RPM = 6500 ;
        }
        else
        {
            MAX_WHEEL_RPM = 4300 ;
        }
    }

    else if(Shift_Flag == 1)
    {
        MAX_WHEEL_RPM = 10000;
    }

    wheel_rpm[0] = (+vx - vy + vz * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[1] = (+vx + vy + vz * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[2] = (-vx + vy + vz * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[3] = (-vx - vy + vz * rotate_ratio_br) * wheel_rpm_ratio;

    //find max item
    for (uint8_t i = 0; i < 4; i++)
    {
        if (fabsf(wheel_rpm[i]) > max)
            max = fabsf(wheel_rpm[i]);
    }

    //equal proportion
    if (max > MAX_WHEEL_RPM)
    {
        float rate = MAX_WHEEL_RPM / max;

        for (uint8_t i = 0; i < 4; i++)
            wheel_rpm[i] *= rate;
    }
	/*********************************************���¼��*************************************************************************************/
//��������ֵ�Ƕ����������ֵ�Ƕ�ƫ�����ʱ��Ϊ����ģʽ����ʱ����ת��Ϊ�ȿ�����ٿ� 

//			buffer_time++;
//			if(buffer_time>BUFFER_TIME)
//			{
#if (INFANTRY_ID==3)	
		if((power_ctrl->chassis_pitch_motor->absolute_angle-power_ctrl->chassis_pitch_motor ->relative_angle)<-13.0f&&
			                                                            (Shift_Flag == 2)&&(power_ctrl->chassis_status_measure->chassis_power_limit<=80))
		{		
		buffer_time++;
			if(buffer_time>500)//����ʱ�䣬С�²�����   ����ƽ�����ݲɼ�����
			{
			  for (uint8_t i = 0; i < 4; i++)
			{
//				wheel_rpm[i]+=wheel_rpm[i]*uphill_ratio;
				abs_limit(&wheel_rpm[i],2200);
////		  wheel_rpm[i]= RAMP_float(wheel_rpm[i],power_ctrl->motor_chassis[i].chassis_motor_measure->speed_rpm,huanchi);//950
			}
		}
		}
		else
		{
		buffer_time=0;
		}
#elif (INFANTRY_ID==4)	
//				if(fabs(power_ctrl->chassis_pitch_motor->absolute_angle-power_ctrl->chassis_pitch_motor ->relative_angle)>max_limit_angle&&
//			                                                            (Shift_Flag == 2)&&(power_ctrl->chassis_status_measure->chassis_power_limit<=80))
//		{	
//				buffer_time++;
//			if(buffer_time>500)//����ʱ�䣬С�²�����   ����ƽ�����ݲɼ�����
//			{
//						  for (uint8_t i = 0; i < 4; i++)
//		 	 {
////				wheel_rpm[i]+=wheel_rpm[i]*uphill_ratio;
//				abs_limit(&wheel_rpm[i],2200);
//////		  wheel_rpm[i]= RAMP_float(wheel_rpm[i],power_ctrl->motor_chassis[i].chassis_motor_measure->speed_rpm,huanchi);//950
//			 }
//	   }
//	}
//				else
//		{
//		buffer_time=0;
//		}
	#elif (INFANTRY_ID==5)	
				if(fabs(power_ctrl->chassis_pitch_motor->absolute_angle-power_ctrl->chassis_pitch_motor ->relative_angle)>max_limit_angle&&
			                                                            (Shift_Flag == 2)&&(power_ctrl->chassis_status_measure->chassis_power_limit<=80))
		{	
			buffer_time++;
						if(buffer_time>500)//����ʱ�䣬С�²�����   ����ƽ�����ݲɼ�����
			{
						  for (uint8_t i = 0; i < 4; i++)
			{
//				wheel_rpm[i]+=wheel_rpm[i]*uphill_ratio;
				abs_limit(&wheel_rpm[i],2200);
////		  wheel_rpm[i]= RAMP_float(wheel_rpm[i],power_ctrl->motor_chassis[i].chassis_motor_measure->speed_rpm,huanchi);//950
			}
		  }
  	}
					else
		{
		buffer_time=0;
		}
#endif

//		  }
	
//		else
//    buffer_time=0;
/***********************************************************************************************************************************************/


    memcpy(speed, wheel_rpm, 4 * sizeof(float));
}


