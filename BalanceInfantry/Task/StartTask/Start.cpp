#include "Start.h"

//任务优先级
#define START_TASK_PRIO 1
//任务堆栈大小
#define START_STK_SIZE 128
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define TASK1_TASK_PRIO 2
//任务堆栈大小
#define TASK1_STK_SIZE 128
//任务句柄
TaskHandle_t Task1Task_Handler;
//任务函数
void task1_task(void *pvParameters);

//任务优先级
#define CHASSIS_TASK_PRIO 18
//任务堆栈大小
#define CHASSIS_TASK_SIZE 128
//任务句柄
TaskHandle_t CHASSISTask_Handler;
//任务函数
void Chassis_task(void *pvParameters);

//任务优先级
#define IMU_TASK_PRIO 30
//任务堆栈大小
#define IMU_STK_SIZE 512
//任务句柄
TaskHandle_t IMUTask_Handler;
//任务函数

void StartTask()
{
	xTaskCreate((TaskFunction_t)start_task,			 //任务函数
				(const char *)"start_task",			 //任务名称
				(uint16_t)START_STK_SIZE,			 //任务堆栈大小
				(void *)NULL,						 //传递给任务函数的参数
				(UBaseType_t)START_TASK_PRIO,		 //任务优先级
				(TaskHandle_t *)&StartTask_Handler); //任务句柄
	vTaskStartScheduler();
}

void start_task(void *pvParameters)
{
	taskENTER_CRITICAL(); //进入临界区
	//创建TASK1任务
	xTaskCreate((TaskFunction_t)task1_task,
				(const char *)"task1_task",
				(uint16_t)TASK1_STK_SIZE,
				(void *)NULL,
				(UBaseType_t)TASK1_TASK_PRIO,
				(TaskHandle_t *)&Task1Task_Handler);
	//创建TASK2任务
	xTaskCreate((TaskFunction_t)Chassis_task,
				(const char *)"Chassis_task",
				(uint16_t)CHASSIS_TASK_SIZE,
				(void *)NULL,
				(UBaseType_t)CHASSIS_TASK_PRIO,
				(TaskHandle_t *)&CHASSISTask_Handler);
	xTaskCreate((TaskFunction_t)IMU_task,
				(const char *)"IMU_task",
				(uint16_t)IMU_STK_SIZE,
				(void *)NULL,
				(UBaseType_t)IMU_TASK_PRIO,
				(TaskHandle_t *)&IMUTask_Handler);
	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();			//退出临界区
}

// task1任务函数
void task1_task(void *pvParameters)
{
	//	u8 task1_num = 0;

	while (1)
	{
		LED0 = !LED0;

		vTaskDelay(1); //延时1s，也就是1000个时钟节拍
	}
}
//--exceptions
// task2任务函数
void BSP_Init()
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //设置系统中断优先级分组4
	delay_init(168);								//初始化延时函数
	uart_init(115200);								//初始化串口
	LED_Init();										//初始化LED端口
	LCD_Init();										//初始化LCD
													//	  while (MPU9250Str.MPU9250_IoInit())
													//  {
													//  }
													//	Mpu9250Init();
	RemoteControlStr.RCInit();
	CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal); // 1Mbps
	CAN2_Mode_Init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);
}
