#include "ADC_Task.h"
#include "math.h"
#include "BSP_ADC_Init.h"
#include "judgement_info.h"
extern receive_judge_t judge_rece_mesg;
Current Real_Current;//裁判系统补偿后的电流值
Current Raw_Current;//实际流经的电流
Current Offest_Current;  //基准值

float ADC1_Get_Channel_renewal[ADC1_Channel_Num];//用于更新的数组 
u8 Time=0;          //采集计数
u8 flag=0;


//ADC数据采集处理
void ADC1_Data_processing(void)
{
  if(flag==0&&Time>=Count)
  {
	   flag=Offest_flag;
		 Time=0;
		 //数据取平均
	   ADC1_Get_Channel_renewal[0]=ADC1_Get_Channel_renewal[0]/Count;
		 ADC1_Get_Channel_renewal[1]=ADC1_Get_Channel_renewal[1]/Count;
		 ADC1_Get_Channel_renewal[2]=ADC1_Get_Channel_renewal[2]/Count;
		 ADC1_Get_Channel_renewal[3]=ADC1_Get_Channel_renewal[3]/Count;
		 ADC1_Get_Channel_renewal[4]=ADC1_Get_Channel_renewal[4]/Count;
		 //得到电压基准
		 Offest_Current.Ch1=ADC1_Get_Channel_renewal[4];		
		 Offest_Current.Ch2=ADC1_Get_Channel_renewal[3];
		 Offest_Current.Ch3=ADC1_Get_Channel_renewal[1];
		 Offest_Current.Ch4=ADC1_Get_Channel_renewal[0];
		 Offest_Current.Ch_Ref=ADC1_Get_Channel_renewal[2];
		
		 ADC1_Get_Channel_renewal[0]=0;
		 ADC1_Get_Channel_renewal[1]=0;
		 ADC1_Get_Channel_renewal[2]=0;
		 ADC1_Get_Channel_renewal[3]=0;
		 ADC1_Get_Channel_renewal[4]=0;
	}
	else if(flag==Offest_flag&&Time>=Count)
	{
  	 Time=0;
		 //数据取平均
		 ADC1_Get_Channel_renewal[0]=ADC1_Get_Channel_renewal[0]/Count;
		 ADC1_Get_Channel_renewal[1]=ADC1_Get_Channel_renewal[1]/Count;
		 ADC1_Get_Channel_renewal[2]=ADC1_Get_Channel_renewal[2]/Count;
		 ADC1_Get_Channel_renewal[3]=ADC1_Get_Channel_renewal[3]/Count;
		 ADC1_Get_Channel_renewal[4]=ADC1_Get_Channel_renewal[4]/Count;
		
		 Raw_Current.Ch1=fabs((Offest_Current.Ch1-ADC1_Get_Channel_renewal[4])*0.00447591);		
		 Raw_Current.Ch2=fabs((Offest_Current.Ch2-ADC1_Get_Channel_renewal[3])*0.00447591);
		 Raw_Current.Ch3=fabs((Offest_Current.Ch3-ADC1_Get_Channel_renewal[1])*0.00447591);
		 Raw_Current.Ch4=fabs((Offest_Current.Ch4-ADC1_Get_Channel_renewal[0])*0.00447591);
		 Raw_Current.Ch_Ref=fabs((Offest_Current.Ch_Ref-ADC1_Get_Channel_renewal[2])*0.00447591);
		
		 Raw_Current.Ch_In=Raw_Current.Ch1+Raw_Current.Ch2+Raw_Current.Ch3+Raw_Current.Ch4;   //通道数据求和
		 Raw_Current.ch_err=judge_rece_mesg.real_powerheat_data.chassisCurrent-Raw_Current.Ch_In;   //与裁判做差
		
		 //与裁判系统进行数据差值互补
		 Real_Current.Ch1=Raw_Current.Ch1+Raw_Current.ch_err*0.25f;
		 Real_Current.Ch2=Raw_Current.Ch2+Raw_Current.ch_err*0.25f;
		 Real_Current.Ch3=Raw_Current.Ch3+Raw_Current.ch_err*0.25f;
		 Real_Current.Ch4=Raw_Current.Ch4+Raw_Current.ch_err*0.25f;
		 Real_Current.Ch_In=Real_Current.Ch1+Real_Current.Ch2+Real_Current.Ch3+Real_Current.Ch4;   //通道数据求和
		 Real_Current.ch_err=judge_rece_mesg.real_powerheat_data.chassisCurrent-Real_Current.Ch_In;   //与裁判做差	
	   //数据清零
		 ADC1_Get_Channel_renewal[0]=0;
		 ADC1_Get_Channel_renewal[1]=0;
		 ADC1_Get_Channel_renewal[2]=0;
		 ADC1_Get_Channel_renewal[3]=0;
		 ADC1_Get_Channel_renewal[4]=0;
	}
	else
  {
		ADC1_Get_Channel_renewal[0]+=ADC1_Real_Channel[0];
		ADC1_Get_Channel_renewal[1]+=ADC1_Real_Channel[1];
		ADC1_Get_Channel_renewal[2]+=ADC1_Real_Channel[2];
		ADC1_Get_Channel_renewal[3]+=ADC1_Real_Channel[3];
		ADC1_Get_Channel_renewal[4]+=ADC1_Real_Channel[4];
		Time++;
	}
}






