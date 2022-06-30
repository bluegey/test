
#include "init.h"
#include  "mydelay.h"
extern Canrxmsg Can_power;
void CAN_Thread1 (void const *argument); //CAN数据发送
osThreadId T_CAN_ID1;
osThreadDef(CAN_Thread1, osPriorityNormal1, 1 ,512);

void PowerMonitor_Thread1 (void const *argument); //电流数据转化接收
osThreadId T_PowerMonitor_ID1;
osThreadDef(PowerMonitor_Thread1, osPriorityNormal1, 1 ,512);

void PowerSupply_Thread1 (void const *argument); //姿态融合 
osThreadId T_PowerSupply_ID1;
osThreadDef(PowerSupply_Thread1, osPriorityNormal1, 1, 512);

float IAN_I_put = 0;
/************************************************/
//创建消息队列
/************************************************/
osSemaphoreId_t GRYO_Semaphore; //得到陀螺仪数据PA2
osSemaphoreDef(GRYO_Semaphore);

osMessageQueueId_t UARTRX_MessageQueue; 
osMessageQDef(UARTRX_MessageQueue,100,u8);

/*RTXKernelStartInit and Task Creak*/
u16 MAX_TIM=1440;
void KernelTaskInit(void)
{
	/************************************************/
	//外设初始化过程
	/************************************************/
	delay_ms(2000);// 延时后系统开始启动
  NVIC_StateInit();
	BSP_GPIO_Init();
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
  BSP_Can_Init();
	BSP_ADC1_Init();
	INA_Init();
	TIM3_PWM_Init(MAX_TIM-1,1-1);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);//超级电容输出通路通
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);//电池输出通路断
	Can_power.power_set=0;
	IAN_I_put = 0;
	TIM_SetCompare4(TIM3,IAN_I_put);
   if(osKernelInitialize()!=osOK)
	 {
		 while(1);
	 }
	 //信号量初始化
//	 GRYO_Semaphore=osSemaphoreNew(1,0,osSemaphore(GRYO_Semaphore)); 
	 
	 T_PowerSupply_ID1=osThreadCreate(osThread(PowerSupply_Thread1),NULL);//供电模式切换任务创建
	 T_CAN_ID1=osThreadCreate(osThread(CAN_Thread1),NULL);	//CAN通信任务创建
   T_PowerMonitor_ID1=osThreadCreate(osThread(PowerMonitor_Thread1),NULL);//功率监测任务创建
   if( osKernelStart()!=osOK) //Start Task Running
	 {
	   while(1);
	 }
}

//中断优先级定义
void NVIC_StateInit(void)
{
    NVIC_InitTypeDef 		Nvic_InitStructure;
  	/* Configure one bit for preemption priority */
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		/*中断设置*/
		Nvic_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   //CAN1 RX0中断
		Nvic_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //抢占优先级0
		Nvic_InitStructure.NVIC_IRQChannelSubPriority = 0;			  		 //子优先级为0
		Nvic_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&Nvic_InitStructure);
		
		Nvic_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;	  	 //CAN1 TX中断
		Nvic_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //抢占优先级0
		Nvic_InitStructure.NVIC_IRQChannelSubPriority = 1;					   //子优先级为0
		Nvic_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&Nvic_InitStructure);
}

//CAN任务发送  
void CAN_Thread1(void const *argument)
{

  while(1)
	{
		CAN_TxMssage();
		LED1_Runsign();
		osDelay(1);
	}
}
float TEST_PWER=12.0f;
u8 flag123=0;
u8 INA_IN = 0,wait = 0;
float voltate = 0,voltate12=23.3f;
//int input_current = 4166;
u8 TTATT =0;
//电流检测
//40	60	80	100
//1.66+100	2.5+100		3.33+100	4.16+100
void PowerMonitor_Thread1(void const *argument)
{
	while(1)
	{
//		if(INAReal_Data.voltage<1800)
//		{
//			Power_Mod_Select(0,0);
//		} 
//		else
//		{
//			Power_Mod_Select(Can_power.power_set,Can_power.flag);
		if(Real_Voltage<=TEST_PWER)
		{ 
			flag123=1;
		   
		}
		else if(Real_Voltage>=22.5)
		{
		 flag123=0;
		}
		if(flag123==0)
		{Power_Mod_Select(100,1);}
		else
		{
			Power_Mod_Select(100,2);
		}
//		}
	}
}
float intcurrent;
void current_select(float current,float power)
{
	if(INA_IN >50)
	{
		INA260_DataUpdate();
		osDelay(2);
			TIM_SetCompare4(TIM3,IAN_I_put);
			osDelay(2);
		if(Real_Voltage<2)
		{
			wait++;
			if(wait>200&&INAReal_Data.current<current)
			{
				wait = 210;
				IAN_I_put = IAN_I_put + 0.1;
			}
			else if(wait>200&&INAReal_Data.current>current)
			{
				wait = 210;
				IAN_I_put = IAN_I_put + 0.1;
			}
			VAL_LIMIT(IAN_I_put,0,MAX_TIM-1);
			TIM_SetCompare4(TIM3,IAN_I_put);
			LED0_Runsign(10);
		}
		else if(Real_Voltage<5&&Real_Voltage>2)
		{
			if(INAReal_Data.current<current)
			{
				IAN_I_put = IAN_I_put + 0.3;
			}
			else if(INAReal_Data.current>current)
			{
				IAN_I_put = IAN_I_put - 0.3;
			}
			VAL_LIMIT(IAN_I_put,0,MAX_TIM-1);
			TIM_SetCompare4(TIM3,IAN_I_put);
			LED0_Runsign(20);
		}
		else if(Real_Voltage<10&&Real_Voltage>5)
		{
			current = (power*1000/(INAReal_Data.voltage+600))*1000;
			if(INAReal_Data.current<current)
			{
				IAN_I_put = IAN_I_put + 0.5;
			}
			else if(INAReal_Data.current>current)
			{
				IAN_I_put = IAN_I_put - 0.5;
			}
			VAL_LIMIT(IAN_I_put,0,MAX_TIM-1);
			LED0_Runsign(40);
			TIM_SetCompare4(TIM3,IAN_I_put);
		}
		else
		{
			current = (power*1000/(INAReal_Data.voltage+600))*1000;
			voltate = INAReal_Data.voltage - 800;
			voltate = (voltate/1000);
			if(voltate>voltate12)
			{
				voltate = voltate12;
			}
			Current_PID(current,voltate);
			if(Real_Voltage<21)
			{
				LED0_Runsign(100);
			}
			else
			{
				LED0_ON;
			}
		}
	}			
}

void Power_Mod_Select(uint8_t inpower,uint8_t flag)
{
	switch(flag)
	{
		case 1:
			GPIO_SetBits(GPIOA,GPIO_Pin_5);//超级电容输出通路通
			GPIO_ResetBits(GPIOA,GPIO_Pin_4);//电池输出通路断
			break;
		case 2:
			GPIO_ResetBits(GPIOA,GPIO_Pin_5);//超级电容输出通路通
			GPIO_ResetBits(GPIOA,GPIO_Pin_4);//电池输出通路通//双通只走电池
			break;
		default:
			GPIO_ResetBits(GPIOA,GPIO_Pin_5);
			GPIO_ResetBits(GPIOA,GPIO_Pin_4);
			break;
	}
	switch(inpower)
	{
		case 0:
			IAN_I_put = 0;
			TIM_SetCompare4(TIM3,IAN_I_put);
			break;
		case 40:
			current_select(1660,40);
			break;
		case 45:
			current_select(1875,45);
		break;
		case 50:
			current_select(2080,50);
		break;
		case 55:
			current_select(2290,55);
		break;
		case 60:
			current_select(2500,60);
			break;
		case 80:
			current_select(3330,80);
			break;
		case 100:
			current_select(4160,100);
			break;
		default:
			if(inpower<100)
			{
				current_select((float)(inpower/24.0f),inpower);
			}
			else
			{
				current_select(0,0);
			}
		break;
	}
}

//0.0005f     0.00009f
//0.0005f     0.000085f
float currentP=0.0005f,currentI=0.000085f,currentD=0;
float voltageP=2000,voltageI=0,voltageD=0;
PID pid_current;
PID pid_voltage;
void Current_PID(int MAXCurrent,float MAXVoltage)
{
	pid_voltage.ref=Real_Voltage;//电容电压
	pid_voltage.fdb=MAXVoltage;//电池输入电压
	PID_Set(&pid_voltage,voltageP,voltageI,voltageD,2000);
	PID_Control(&pid_voltage);
	VAL_LIMIT(pid_voltage.pid_out,-MAXCurrent,MAXCurrent);
	
	pid_current.ref=-pid_voltage.pid_out;
	pid_current.fdb=INAReal_Data.current;
	PID_Set(&pid_current,currentP,currentI,currentD,2000);
	PID_Control(&pid_current);
	VAL_LIMIT(pid_current.pid_out,-MAX_TIM+1,MAX_TIM-1);
	IAN_I_put = IAN_I_put + pid_current.pid_out;
//	IAN_I_put = pid_current.pid_out;
	VAL_LIMIT(IAN_I_put,0,MAX_TIM-3);
	TIM_SetCompare4(TIM3,IAN_I_put);
}


//供电方式切换
void PowerSupply_Thread1(void const *argument)
{	  u16 adc_origin;
	while(1)
	{
		adc_origin=Get_Adc_Average(ADC_Channel_1,10);
		adc_voltage=(float)adc_origin*(3.3f/4096);
//		Real_Voltage=adc_voltage*7/0.915f;//这是之前的二极管加mos的
		Real_Voltage=adc_voltage*7.62;//这是理想二极管的板子
//		if(Real_Voltage>18)
//		{
//			LED0_Runsign(30);
//		}
		INA_IN=INA_IN+1;
		if(INA_IN>50)
		{
			INA_IN = 51;
		}
		osDelay(2);
	}
}
