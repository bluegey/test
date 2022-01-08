/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "tim.h"
#include "PID_Control.h"
#include "INA260_Driver.h" 
#include "fdcan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
u8 volt_flag=0;
InaReal_Data  INAReal_Data={0};
Can_Rx_Union  Can_Rx_Data;
int MAX_TIM=1103;
float adc_voltage,Real_Voltage,TEST_PWER=12.0f,IAN_I_put=0,voltate = 0,voltate12=23.3f;
unsigned char INA_IN = 0,wait = 0,flag123=0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t Send_Data[sizeof(InaReal_Data)]={0},*Tx_Change_Dtat;
uint8_t Rx_data[8];

void Power_Mod_Select(uint8_t inpower,uint8_t flag);
void current_select(float current,float power);
void Current_PID(int MAXCurrent,float MAXVoltage);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId ADCTaskHandle;
osThreadId Power_TaskHandle;
osThreadId CAN_TaskHandle;
osMessageQId myQueue01Handle;
osMessageQId myQueue02Handle;
osMessageQId myQueue03Handle;
osTimerId myTimer01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Adc_Task(void const * argument);
void Start_Power_Task(void const * argument);
void Start_Can_Task(void const * argument);
void Callback01(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 16, uint16_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* definition and creation of myQueue02 */
  osMessageQDef(myQueue02, 16, uint16_t);
  myQueue02Handle = osMessageCreate(osMessageQ(myQueue02), NULL);

  /* definition and creation of myQueue03 */
  osMessageQDef(myQueue03, 16, uint16_t);
  myQueue03Handle = osMessageCreate(osMessageQ(myQueue03), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ADCTask */
  osThreadDef(ADCTask, Start_Adc_Task, osPriorityNormal, 0, 128);
  ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);

  /* definition and creation of Power_Task */
  osThreadDef(Power_Task, Start_Power_Task, osPriorityNormal, 0, 256);
  Power_TaskHandle = osThreadCreate(osThread(Power_Task), NULL);

  /* definition and creation of CAN_Task */
  osThreadDef(CAN_Task, Start_Can_Task, osPriorityNormal, 0, 128);
  CAN_TaskHandle = osThreadCreate(osThread(CAN_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start_Adc_Task */
/**
  * @brief  Function implementing the ADCTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Adc_Task */
u8 Adc_flag=0;
void Start_Adc_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Adc_Task */
  /* Infinite loop */
  for(;;)
  {
    HAL_ADC_Start(&hadc2);
		if(HAL_ADC_PollForConversion(&hadc2,0xff)==HAL_OK)
			{
				Adc_flag=1;
			 adc_voltage=HAL_ADC_GetValue(&hadc2)*3.3/4096;
			}
			else
			{
			  Adc_flag=2;
				LED0();
				LED1();
			}
		Real_Voltage=adc_voltage*7.62f;
		INA_IN=INA_IN+1;
		if(INA_IN>50)
		{
			INA_IN = 51;
		}		
    osDelay(1);
  }
  /* USER CODE END Start_Adc_Task */
}

/* USER CODE BEGIN Header_Start_Power_Task */
/**
* @brief Function implementing the Power_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Power_Task */
void Start_Power_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Power_Task */
	INA_Init();
  /* Infinite loop */
  for(;;)
  {
#if (TEST_IS_OR_NOT==1)
		if(Can_Rx_Data.Rx_Data.flag ==1)
		{
		 Power_Mod_Select(Can_Rx_Data.Rx_Data .power_limit ,1);
		}
		else if(Can_Rx_Data.Rx_Data.flag ==2)
		{
		Power_Mod_Select(Can_Rx_Data.Rx_Data .power_limit ,2);
		}
		else
    {
		Power_Mod_Select(40 ,0);
		}
#elif (TEST_IS_OR_NOT==0)
		if(Real_Voltage<=TEST_PWER)
		{ 
			flag123=1;
		   
		}
		else if(Real_Voltage>=22.5f)
		{
		  flag123=0;
		}
		if(flag123==0)
		{
		  Power_Mod_Select(100,1);
		}
		else
		{
			Power_Mod_Select(100,2);
		}
#endif
    
    osDelay(1);
  }
  /* USER CODE END Start_Power_Task */
}

/* USER CODE BEGIN Header_Start_Can_Task */

/**
* @brief Function implementing the CAN_Task thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_Start_Can_Task */
Tx_data Can_sent_data;
u16 err_time=0;
void Start_Can_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Can_Task */
  /* Infinite loop */
  for(;;)
  {
		Can_sent_data.super_volt=(u16)(Real_Voltage*1000);
		Can_sent_data.power=INAReal_Data.power;
		Can_sent_data.current =INAReal_Data.current;
		Tx_Change_Dtat=(uint8_t *)&Can_sent_data;
		for(int i=0;i<sizeof(Tx_data);i++)
		{
		  Send_Data[i]=*Tx_Change_Dtat;
			Tx_Change_Dtat++;
		}
		Send_Massage(0x300,Send_Data);
	   osDelay(1);		
		 if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&RxHeader,Can_Rx_Data.Array_RX_data)==HAL_OK&&Adc_flag==1)
		 {
			 err_time=0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		 }
		 else if(Adc_flag==1&&HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&RxHeader,Can_Rx_Data.Array_RX_data)!=HAL_OK)
		 {
			 err_time++;
			 if(err_time>=500)
			 { LED1();err_time=500;}
		 }
    osDelay(1);
  }
  /* USER CODE END Start_Can_Task */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
   void Power_Mod_Select(uint8_t inpower,uint8_t flag)
{
	switch(flag)
	{
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);//超级电容输出通路通
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);//电池输出通路断
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);//超级电容输出通路通
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);//电池输出通路通//双通只走电池

			break;
		default:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
	}
	switch(inpower)
	{
		case 0:
			IAN_I_put = 0;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, IAN_I_put);
			break;
		case 40:
			current_select(1660,40);//
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
float change_rate;
void current_select(float current,float power)
{
	if(INA_IN >50)
	{
		
		INA260_DataUpdate();
			osDelay(2);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, IAN_I_put);
				osDelay(2);
		if(Real_Voltage<2)
		{
			wait++;
			if(wait>200&&INAReal_Data.current<current)
			{
				wait = 210;
				IAN_I_put = IAN_I_put + 0.1f;
			}
			else if(wait>200&&INAReal_Data.current>current)
			{
				wait = 210;
				IAN_I_put = IAN_I_put - 0.1f;
			}
			VAL_LIMIT(IAN_I_put,0,MAX_TIM);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, IAN_I_put);
			volt_flag=1;
		}
		else if(Real_Voltage<5&&Real_Voltage>2)
		{
			if(INAReal_Data.current<current)
			{
				IAN_I_put = IAN_I_put + 0.3f;
			}
			else if(INAReal_Data.current>current)
			{
				IAN_I_put = IAN_I_put - 0.3f;
			}
			VAL_LIMIT(IAN_I_put,0,MAX_TIM);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, IAN_I_put);
			volt_flag=1;
		}
		else if(Real_Voltage<10&&Real_Voltage>5)
		{
			current = (power*1000/(INAReal_Data.voltage+600))*1000;
			if(INAReal_Data.current<current)
			{
				IAN_I_put = IAN_I_put + 0.5f;
			}
			else if(INAReal_Data.current>current)
			{
				IAN_I_put = IAN_I_put - 0.5f;
			}
			VAL_LIMIT(IAN_I_put,0,MAX_TIM);
			volt_flag=1;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, IAN_I_put);
		}
		else
		{
			change_rate=((float)Can_Rx_Data.Rx_Data.Buffer_Energy*1000/((float)INAReal_Data.voltage+600))*1000*((float)Can_Rx_Data.Rx_Data.Buffer_Energy/60);
			current = (power*1000/(INAReal_Data.voltage+600))*1000/*+(Can_Rx_Data.Rx_Data.Buffer_Energy*1000/(INAReal_Data.voltage+600))*1000*(Can_Rx_Data.Rx_Data.Buffer_Energy/60)*/;
			voltate = INAReal_Data.voltage - 800;
			voltate = (voltate/1000);
			if(voltate>voltate12)
			{
				voltate = voltate12;
			}
			Current_PID(current,voltate);
			if(Real_Voltage<21)
			{
				volt_flag=1;
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			}
			else
			{
				volt_flag=2;
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			}
		}
	}			
}
float currentP=0.0005f,currentI=0.000085f,currentD=0;
float voltageP=2000,voltageI=0,voltageD=0;
PID pid_current;
PID pid_voltage;
void Current_PID(int MAXCurrent,float MAXVoltage)
{
	pid_voltage.ref=Real_Voltage;
	pid_voltage.fdb=MAXVoltage;
	PID_Set(&pid_voltage,voltageP,voltageI,voltageD,2000);
	PID_Control(&pid_voltage);//电容电压-输入电压
	VAL_LIMIT(pid_voltage.pid_out,-MAXCurrent,MAXCurrent);//限电流
	
	pid_current.ref=-pid_voltage.pid_out;//
	pid_current.fdb=INAReal_Data.current;//输入电流
	PID_Set(&pid_current,currentP,currentI,currentD,2000);
	PID_Control(&pid_current);//-电压环输出-输入电流
	VAL_LIMIT(pid_current.pid_out,-MAX_TIM,MAX_TIM);
	IAN_I_put = IAN_I_put + pid_current.pid_out;
	VAL_LIMIT(IAN_I_put,0,MAX_TIM);
	
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, IAN_I_put);
}  
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
