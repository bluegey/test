#include "main.h"

#define Start_Task_Prio  1
#define Start_Task_Size 128
static TaskHandle_t StartTask_Handle;
void Start_Task(void *pvParameters);

#define OLED_Task_Prio 2
#define OLED_Task_Size 128
static TaskHandle_t OLED_Task_Handle;

#define Motor_Task_Prio 3
#define Motor_Task_Size 128
static TaskHandle_t Motor_Task_Handle;
void Motor_Task(void *pvParameters);

#define Steer_Task_Prio 3
#define Steer_Task_Size 128
static TaskHandle_t Steer_Task_Handle;
void Steer_Task(void *pvParameters);

void Start_Task(void *pvParameters)
{
	taskENTER_CRITICAL();
	
	xTaskCreate((TaskFunction_t )OLED_Task, 
				(const char*    )"OLED_Task", 
				(uint16_t       )OLED_Task_Prio, 
				(void*          )NULL, 
				(UBaseType_t    )OLED_Task_Prio,  
				(TaskHandle_t*  )&OLED_Task_Handle);
				
	xTaskCreate((TaskFunction_t )Motor_Task, 
				(const char*    )"Motor_Task", 
				(uint16_t       )Motor_Task_Size, 
				(void*          )NULL, 
				(UBaseType_t    )Motor_Task_Prio,  
				(TaskHandle_t*  )&Motor_Task_Handle);
	xTaskCreate((TaskFunction_t )Steer_Task, 
				(const char*    )"Steer_Task", 
				(uint16_t       )Steer_Task_Size, 
				(void*          )NULL, 
				(UBaseType_t    )Steer_Task_Prio,  
				(TaskHandle_t*  )&Steer_Task_Handle);

	vTaskDelete(StartTask_Handle); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}
/*�����ʼ��*/
void StartTask_Init(void)   
{
	//������ʼ����
    xTaskCreate((TaskFunction_t )Start_Task,            //������
                (const char*    )"Start_Task",          //��������
                (uint16_t       )Start_Task_Size,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )Start_Task_Prio,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handle);   //������              
    vTaskStartScheduler();          //�����������
}
/*�����ʼ��*/
void Module_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	uart1_init(100000);
	delay_init();
	KEY_Init();//KEY��ʼ��
//	LED_Init();//LED��ʼ��
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,2,CAN_Mode_Normal);//,������1000Kbps  
	  OLED_Init();//OLED��ʼ��
		 
}
