#include "oled_task.h"

oled_control_t oled_control;
extern fp32 TIME_COUNT0;
fp32 Voltage;
int CAN_flag,rc_flag,angle_flag,erroe_flag=0;

void oled_task(void *pvParameters)
{	
  oled_init(&oled_control);
	vTaskDelay(60);
	
		for(int i=0;i<monitorLength;i++)
	{
		 if(toe_is_error(i)==1)
		{
			erroe_flag++;
		}
	}
	
	while(1)
	{

	if(erroe_flag!=0)beep_detect(erroe_flag);
	else beep_detect(0);
	
		oled_Feedback_Update(&oled_control);
		oled_running(&oled_control);
		vTaskDelay(1); 
	}
}

static void oled_init(oled_control_t *oled_init)
{
	oled_init->gimbal_monitor_point = getErrorListPoint();;
	oled_init->gimbal_angle_gyro_point = get_Gyro_Angle_Point();
	oled_init->GET_rc_point = get_remote_control_point();
	oled_init->mid_time=1;
}
static void oled_Feedback_Update(oled_control_t *oled_feedback)
{

	
	oled_feedback->Anglex = oled_feedback->gimbal_angle_gyro_point->YAW;
	oled_feedback->Angley = oled_feedback->gimbal_angle_gyro_point->Pitch;
	oled_feedback->Anglez = oled_feedback->gimbal_angle_gyro_point->ROLL;

  oled_feedback->Gyrox = oled_feedback->gimbal_angle_gyro_point->V_X;
  oled_feedback->Gyroy = oled_feedback->gimbal_angle_gyro_point->V_Y;
	oled_feedback->Gyroz = oled_feedback->gimbal_angle_gyro_point->V_Z;
	
  oled_feedback->oled_time = TIME_COUNT0 + oled_feedback->mid_time;
	oled_feedback->oled_VOLTAGE = Get_Adc_Average(Battery_Ch,10);
	oled_feedback->oled_voltage = oled_feedback->oled_VOLTAGE/(4096)*9.5*3.3;  
}

int error_count=0;
static void oled_running(oled_control_t *oled_go)
{
static int error_count=0;
	OLED_DClean();
	oled_show(oled_go);
	if(oled_go->oled_time>12)
	{
		
		for(int i=0;i<monitorLength;i++)
	{
//		if(!(toe_is_error(i)))
		 if(toe_is_error(i)==1)
		{
			error_count++;

		}
	}
	if(error_count==0)
						OLED_printf(10,0,"no");
	else if(error_count!=0)
	{	
		TIME_COUNT0=5;
		oled_show(oled_go);}


}
}

static void oled_show(oled_control_t *oled_show)
{
static int gyro_error=0;
	
//	OLED_printf(0,0,"mode");

	
if(oled_show->oled_time<5)
{
	OLED_printf(20,10,"preparing...");

	if(myabs(oled_show->Anglex)>0.1)
	gyro_error=1;
	else gyro_error=0;
}
else if(oled_show->oled_time>=5)
{
	OLED_printf(70,0,"error");
	OLED_printf(10,10,"gyro");
	OLED_printf(10,25,"V");
	OLED_printf(70,25,"%.3f",oled_show->oled_voltage);
	OLED_printf(100,25,"v");
  if(gyro_error!=0)	OLED_printf(70,10,"error");
	else if(gyro_error==0)OLED_printf(70,10,"normal");
}

	switch(oled_show->oled_time)
	{
		case 5:
	if(toe_is_error(YawGimbalMotorTOE))
	{
		OLED_printf(10,0,"yaw");
	}
	else
		TIME_COUNT0+=oled_show->mid_time;break;
		case 6:
	if(toe_is_error(PitchGimbalMotorTOE))
	{
		OLED_printf(10,0,"pitch");
	}
	else
		TIME_COUNT0+=oled_show->mid_time;break;
		case 7:
	if(toe_is_error(Chassis1WDG)||toe_is_error(Chassis2WDG)||toe_is_error(Chassis3WDG)||toe_is_error(Chassis4WDG))
	{
		OLED_printf(10,0,"chassis");
	}
	else
		TIME_COUNT0+=oled_show->mid_time;break;
			case 8:
	if(toe_is_error(frictionmotorRTOE)||toe_is_error(frictionmotorLTOE))
	{
		OLED_printf(10,0,"friction");
	}
	else
		TIME_COUNT0+=oled_show->mid_time;break;
			case 9:
	if(toe_is_error(TriggerWDG))
	{
		OLED_printf(10,0,"trigger");
	}
	else
		TIME_COUNT0+=oled_show->mid_time;break;
			case 10:
	if(toe_is_error(DBUSWDG))
	{
		OLED_printf(10,0,"DBUSWDG");
	}
	else
		TIME_COUNT0+=oled_show->mid_time;break;
			case 11:
	if(toe_is_error(JudgementWDG))
	{
		OLED_printf(10,0,"judgement");
	}
	else
		TIME_COUNT0+=oled_show->mid_time;break;
			case 12:
	if(toe_is_error(TX2DataWGD))
	{
		OLED_printf(10,0,"TX2");
	}
	else
		TIME_COUNT0+=oled_show->mid_time;break;
	default:
		break;

//				beep_detect(3);

}
	

	
	OLED_Display();
}



static void beep_detect(int time)
{
static int32_t all_time=0,cnt=0,count=0,time0=0,time1=0;
	all_time++;
	time1++;
	if(all_time>20)
	{
		TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//开启中断，响
		cnt = cnt+3;
		all_time =0;
	}
	if(cnt==6)
	{
		count++;
		TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);//关闭中断，停
		cnt=0;

	}
	if(count>time-1)
	{
		TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);//关闭中断，停
		all_time =0;
		time0++;
		if(time0>300)
		{
			time0=0;
			count=0;
		}
	}
	if(time1>5000)
	{
		TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);//关闭中断，停
	all_time=0;count=0;}
}


//绝对值函数
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}

