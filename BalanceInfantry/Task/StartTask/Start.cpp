#include "Start.h"

//�������ȼ�
#define START_TASK_PRIO 1
//�����ջ��С
#define START_STK_SIZE 128
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

//�������ȼ�
#define TASK1_TASK_PRIO 2
//�����ջ��С
#define TASK1_STK_SIZE 128
//������
TaskHandle_t Task1Task_Handler;
//������
void task1_task(void *pvParameters);

//�������ȼ�
#define CHASSIS_TASK_PRIO 18
//�����ջ��С
#define CHASSIS_TASK_SIZE 128
//������
TaskHandle_t CHASSISTask_Handler;
//������
void Chassis_task(void *pvParameters);

//�������ȼ�
#define IMU_TASK_PRIO 30
//�����ջ��С
#define IMU_STK_SIZE 512
//������
TaskHandle_t IMUTask_Handler;
//������

void StartTask()
{
	xTaskCreate((TaskFunction_t)start_task,			 //������
				(const char *)"start_task",			 //��������
				(uint16_t)START_STK_SIZE,			 //�����ջ��С
				(void *)NULL,						 //���ݸ��������Ĳ���
				(UBaseType_t)START_TASK_PRIO,		 //�������ȼ�
				(TaskHandle_t *)&StartTask_Handler); //������
	vTaskStartScheduler();
}

void start_task(void *pvParameters)
{
	taskENTER_CRITICAL(); //�����ٽ���
	//����TASK1����
	xTaskCreate((TaskFunction_t)task1_task,
				(const char *)"task1_task",
				(uint16_t)TASK1_STK_SIZE,
				(void *)NULL,
				(UBaseType_t)TASK1_TASK_PRIO,
				(TaskHandle_t *)&Task1Task_Handler);
	//����TASK2����
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
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
	taskEXIT_CRITICAL();			//�˳��ٽ���
}

// task1������
void task1_task(void *pvParameters)
{
	//	u8 task1_num = 0;

	while (1)
	{
		LED0 = !LED0;

		vTaskDelay(1); //��ʱ1s��Ҳ����1000��ʱ�ӽ���
	}
}
//--exceptions
// task2������
void BSP_Init()
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //����ϵͳ�ж����ȼ�����4
	delay_init(168);								//��ʼ����ʱ����
	uart_init(115200);								//��ʼ������
	LED_Init();										//��ʼ��LED�˿�
	LCD_Init();										//��ʼ��LCD
													//	  while (MPU9250Str.MPU9250_IoInit())
													//  {
													//  }
													//	Mpu9250Init();
	RemoteControlStr.RCInit();
	CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal); // 1Mbps
	CAN2_Mode_Init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);
}
