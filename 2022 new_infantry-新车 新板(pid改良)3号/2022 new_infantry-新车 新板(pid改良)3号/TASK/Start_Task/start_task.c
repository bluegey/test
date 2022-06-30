#include "main.h"
#include "start_task.h"
#define START_TASK_PRIO		1
#define START_STK_SIZE 		128
static TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

#define IMU_TASK_PRIO 32
#define IMU_TASK_SIZE 512
static TaskHandle_t MPUTask_Handler;

#define GIMBAL_TASK_PRIO 18
#define GIMBAL_TASK_SIZE 256
static TaskHandle_t GIMBALTask_Handler;

#define CHASSIS_TASK_PRIO 18
#define CHASSIS_TASK_SIZE 256
static TaskHandle_t CHASSISTask_Handler;

#define SHOOT_TASK_PRIO 17
#define SHOOT_TASK_SIZE 256
static TaskHandle_t SHOOTTask_Handler;

#define TX2_TASK_PRIO 20
#define TX2_TASK_SIZE 256
static TaskHandle_t TX2_TASK_Handler;

#define JUDGE_TASK_PRIO 19
#define JUDGE_TASK_SIZE 256
static TaskHandle_t JUDGE_TASK_Handler;

#define INTER_TASK_PRIO 16
#define INTER_TASK_SIZE 512
static TaskHandle_t INTER_TASK_Handler;

#define DETECT_TASK_PRIO 10
#define DETECT_TASK_SIZE 128
static TaskHandle_t DETECT_TASK_Handler;

//#define OLED_TASK_PRIO 9
//#define OLED_TASK_SIZE 128
//static TaskHandle_t OLED_TASK_Handler;
//extern void oled_task(void *pvParameters);
#define UI_TASK_PRIO 17
#define UI_TASK_SIZE 512
TaskHandle_t UI_TASK_Handler;

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    //陀螺仪任务
//    xTaskCreate((TaskFunction_t )IMU_task,
//                (const char*    )"IMU_task",
//                (uint16_t       )IMU_TASK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )IMU_TASK_PRIO,
//                (TaskHandle_t*  )&MPUTask_Handler);
    //云台任务
    xTaskCreate((TaskFunction_t )GIMBAL_task,
                (const char*    )"GIMBAL_task",
                (uint16_t       )GIMBAL_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )GIMBAL_TASK_PRIO,
                (TaskHandle_t*  )&GIMBALTask_Handler);
    //底盘任务
    xTaskCreate((TaskFunction_t )chassis_task,
                (const char*    )"chassis_task",
                (uint16_t       )CHASSIS_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )CHASSIS_TASK_PRIO,
                (TaskHandle_t*  )&CHASSISTask_Handler);
    //射击任务
    xTaskCreate((TaskFunction_t )shoot_task,
                (const char*    )"shoot_task",
                (uint16_t       )SHOOT_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )SHOOT_TASK_PRIO,
                (TaskHandle_t*  )&SHOOTTask_Handler);
    //TX2任务
    xTaskCreate((TaskFunction_t )Tx2_task,
                (const char*    )"Tx2_task",
                (uint16_t       )TX2_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TX2_TASK_PRIO,
                (TaskHandle_t*  )&TX2_TASK_Handler);
    //裁判系统任务
    xTaskCreate((TaskFunction_t )Judge_task,
                (const char*    )"Judge_task",
                (uint16_t       )JUDGE_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )JUDGE_TASK_PRIO,
                (TaskHandle_t*  )&JUDGE_TASK_Handler);
    //MINIPC发送任务
    xTaskCreate((TaskFunction_t )Interactive_task,
                (const char*    )"Interactive_task",
                (uint16_t       )INTER_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )INTER_TASK_PRIO,
                (TaskHandle_t*  )&INTER_TASK_Handler);
    //监测任务
    xTaskCreate((TaskFunction_t )detect_task,
                (const char*    )"detect_task",
                (uint16_t       )DETECT_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )DETECT_TASK_PRIO,
                (TaskHandle_t*  )&DETECT_TASK_Handler);
		//oled任务
//    xTaskCreate((TaskFunction_t )oled_task,
//                (const char*    )"oeld_task",
//                (uint16_t       )OLED_TASK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )OLED_TASK_PRIO,
//                (TaskHandle_t*  )&OLED_TASK_Handler);
    //UI
    xTaskCreate((TaskFunction_t )ui_task,
                (const char*    )"ui_task",
                (uint16_t       )UI_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )UI_TASK_PRIO,
                (TaskHandle_t*  )&UI_TASK_Handler);

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

/*
*********************************************************************************************************
* StartTask_Handler
*********************************************************************************************************
*/
static TaskHandle_t StartTask_Handler;
void startTask(void)
{
    //创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄
    vTaskStartScheduler();          //开启任务调度
}

/********************************BSP_Init***********************************/
void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
    delay_init(168);//初始化延时函数
	delay_ms(2000);
    TIM5_PWM_Init(20000 - 1, 84 - 1); //84M/84/20000=50hz. 此处的200就是计数器的比较值 e.g.175/200*20ms=17.5ms    得脉冲宽度：20ms-17.5ms=2.5ms
    Device_Usart1_ENABLE_Init(115200);//初始化裁判系统通讯串口
    Device_Usart6_ENABLE_Init(921600);//初始化视觉通讯串口
    //初始化LED端口和激光
    LED_Init();
//	  OLED_Init();
    LASER_Init();
    //遥控器初始化
    remote_control_init();
    //CAN接口初始化//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);//1Mbps
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);
	  Adc_Init();
			TIM3_Int_Init(29999,8400-1);//一秒灯
//		TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);//关闭中断，停
    //MPU初始化
//    while(MPU6500_Init());
//    IMU_Calibration();
//    accel_mat_init();
//    ACCEL_Calibration();
//    MPUHEAT_configuration();

}
