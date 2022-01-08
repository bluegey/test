#ifndef __OLED_TASK_H
#define __OLED_TASK_H

#include "sys.h"
#include "Can_Receive.h"
#include "RC.h"
#define MID_DATA       76
#define INIT_DATA      39
#define INTERVAL_DATA  8 
#define INIT2_DATA     INIT_DATA+50 
//enum KEY_Mode
//{
//

//};

#define TEST  1
void fun0(void);
void fun1(void);
void fun2(void);
//void fun3(void);
void fun4(void);
void fun5(void);
void fun6(void);
void fun7(void);
void fun8(void);
void fun9(void);
void fun10(void);
void fun11(void);
void fun12(void);
void fun13(void);
void fun14(void);
void fun15(void);
void fun16(void);
void fun17(void);
void fun18(void);
void fun19(void);
void fun20(void);
typedef struct
{
    uint8_t current;//当前状态索引号
	  uint8_t up;//向上按
    uint8_t enter;//确定键按 
    uint8_t next;//向下按
	  uint8_t Return;
    void (*current_operation)(void);//当前状态应该执行的操作
}Menu_table;
typedef struct
{
	float temperature;
	float yaw;
	float pitch;
	float roll;
}Mpu_feedback;

typedef struct 
{
const motor_measure_t *point_motor[4];
int16_t display_data;//设定目标值
//int16_t fdb_spm;//电机反馈速度
//int16_t fdb_ecd;//电机反馈转子角度
//u8 id[1];
u8 fpb_display[4];	
int_least8_t  display[4];//将目标值显示在oled使用
const RC_ctrl_t *RC_data;	
//char Fdb_spm[4];//用于将反馈速度显示在oled上
//char Fdb_ecd[4];//用于将电机反馈转子机械角度显示在oled使用
}Dispay_Motor;

extern void OLED_Task(void *pvParameters);

#endif
