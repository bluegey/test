#include "OLED_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"
#include "oled.h"
#include "key.h"
#include "OLED_Task.h"
#include "remote.h"
#include "pid.h"
#define uchar unsigned char
/*===执行函数====*/
void (*current_operation_index)(); //执行当前操作函数
uint8_t func_index = 0;
/*===OLED显示===*/
static void OLED_StartDisplay(void);//开始界面
static void OLED_MainMenuDisplay(void);//主菜单选择
/*===OLED电机部分的显示===*/
static void OLED_MotorMenuDisplay(void);//选择电机界面
//static void OLED_MotorModeDisplay(void);//电机工作方式的选择
static void OLED_MotorSetDisplay(void);//电机性能设置界面
static void OLED_MotorFeedbackDisplay(void);//电机反馈结果界面
/*===按键执行任务===*/
static void KEY_WorkSet1(void);
static void KEY_WorkSet2(void);
static void KEY_WorkSet3(void);
static void KEY_WorkSet4(void);
void change_data(int x,int y,char *data);
void Motor_play(int16_t data_play);
void char_display(u8 x,u8 y,u8 deviation,u8 length,char *data);
/*===PID调参===*/
Mpu_feedback mpu_data_feed;
uint16_t Motor_Set= 0; 	
uint16_t Motor_SetNum[]={0,0};
uint16_t Motor_ID[]={0,1,2,3,4,5,6};
Dispay_Motor motor_diaplay;
int_least8_t id=1;
int_least8_t subscript=0;
//u8 ide[2]="00";
int_least8_t angle[3]={0};
uint16_t Angle=0;
u8 steer_flag=0,subscript_min=0;
char ceshi[][16]={
0x00,0xFC,0x44,0x44,0xC4,0x38,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x07,0x00,0x00,//"R",0   terminal
0x00,0xC0,0xA0,0xA0,0xA0,0xC0,0x00,0x00,0x00,0x03,0x04,0x04,0x04,0x02,0x00,0x00,//"e",1
0x00,0xE0,0x20,0xC0,0x20,0xC0,0x00,0x00,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x00,//"m",2
0x00,0xC0,0x20,0x20,0x20,0xC0,0x00,0x00,0x00,0x03,0x04,0x04,0x04,0x03,0x00,0x00,//"o",3
0x00,0x20,0xFC,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x04,0x04,0x00,0x00,0x00,//"t",4
0x00,0xC0,0xA0,0xA0,0xA0,0xC0,0x00,0x00,0x00,0x03,0x04,0x04,0x04,0x02,0x00,0x00,//"e",5
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//" ",6
0x00,0x20,0xFC,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x04,0x04,0x00,0x00,0x00,//"t",7
0x00,0x40,0xA0,0xA0,0xC0,0x00,0x00,0x00,0x00,0x03,0x04,0x04,0x03,0x04,0x00,0x00,//"a",8
0x00,0x40,0xA0,0xA0,0x20,0x40,0x00,0x00,0x00,0x02,0x04,0x04,0x05,0x02,0x00,0x00,//"s",9
0x00,0xFC,0x00,0x80,0x40,0x20,0x00,0x00,0x00,0x07,0x01,0x01,0x02,0x04,0x00,0x00,//"k",10
};
char mpu[][16]={
{0x00,0x04,0x04,0xFC,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00},//T
{0x00,0xFC,0x44,0x44,0x44,0x04,0x00,0x00,0x00,0x07,0x04,0x04,0x04,0x04,0x00,0x00},//E
{0x00,0xFC,0x70,0x80,0x70,0xFC,0x00,0x00,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x00},//M
	
{0x00,0x0C,0x30,0xC0,0x30,0x0C,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00},//Y
{0x00,0x80,0x70,0x0C,0x70,0x80,0x00,0x00,0x00,0x07,0x01,0x01,0x01,0x07,0x00,0x00},//A
{0x00,0x7C,0x80,0x7C,0x80,0x7C,0x00,0x00,0x00,0x00,0x07,0x00,0x07,0x00,0x00,0x00},//W

{0x00,0xFC,0x44,0x44,0x44,0x38,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00},//P
{0x00,0x00,0x04,0xFC,0x04,0x00,0x00,0x00,0x00,0x00,0x04,0x07,0x04,0x00,0x00,0x00},//I
{0x00,0x04,0x04,0xFC,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00},//T

{0x00,0xFC,0x44,0x44,0xC4,0x38,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x07,0x00,0x00},//R
{0x00,0xF8,0x04,0x04,0x04,0xF8,0x00,0x00,0x00,0x03,0x04,0x04,0x04,0x03,0x00,0x00},//O
{0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x04,0x04,0x04,0x04,0x00,0x00},//L
{0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x04,0x04,0x04,0x04,0x00,0x00},//L

{0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x06,0x00,0x00,0x00,0x00},//:
	};
char steer_motor[8][11]={
{0x70,0x88,0x88,0x70,0x07,0x00,0x00,0x00,0x00,0x00,0x00},//0
{0x70,0x88,0x88,0x70,0x00,0x07,0x80,0x00,0xB8,0xA8,0xE8},//0.5
{0x00,0xF8,0x00,0x00,0x80,0x07,0x00,0x70,0x88,0x88,0x70},//1.0
{0x00,0xF8,0x00,0x00,0x80,0x07,0x00,0x00,0xB8,0xA8,0xE8},//1.5
{0x98,0xC8,0xA8,0x98,0x00,0x87,0x00,0x70,0x88,0x88,0x70},//2.0

};
Menu_table table[21]=
{
	{0,1,1,1,1,(*OLED_StartDisplay)},//0 初始界面  initital interface
//主菜单	
	{1,4,5,2,1,(*fun0)},//1     光标指向电机选项    the curcos points to the motor option
	{2,1,13,3,3,(*fun1)},//2     光标指向陀螺仪选项  the curcos points to the gyro option 
	{3,2,12,4,3,(*fun2)},//2    光标指向遥控器选项  the curcos points to the remote option
	{4,3,19,1,4,(*fun4)},//4     光标指向串口选项
/***********************电机菜单****************************************/
//电机一级菜单	
	{5,8,9,6,1,(*fun5)},//2     光标指向舵机选项    the curcos points to the can option
	{6,5,9,7,1,(*fun6)},//4     光标指向3508电机选项 the curcos points to the 3508 motor option 
	{7,6,9,8,1,(*fun7)},//5     光标指向6020电机选项 the curcos points to the 6020 motor option
  {8,7,14,5,1,(*fun8)},//6     光标指向2006电机选项 the curcos points to the 2006 motor option 
//电机二级菜单	
	{9,9,10,9,5,(*fun9)},//7     设置电调id
	{10,10,11,10,5,(*fun10)},//8    设置转速
//电机三级菜单
	{11,11,11,11,5,(*fun11)},//9  显示电机反馈速度和设定转速
  
/****************************遥控器菜单***********************************/
  {12,12,12,12,3,(*fun12)},	
	
/****************************陀螺仪显示***********************************/
  {13,13,13,13,2,(*fun13)},
/*****************************舵机设置**************************************/
//舵机二级菜单
  {14,16,17,15,8,(*fun14)},//模拟舵机
	{15,14,17,16,8,(*fun15)},//数字舵机
	{16,15,16,14,8,(*fun16)},//其他
	
//舵机三级菜单
	{17,17,18,17,14,(*fun17)},//
//舵机四级菜单
	
	{18,18,18,18,14,(*fun18)},
//串口调试界面显示
  {19,19,20,19,4,(*fun19)},
	{20,20,20,20,19,(*fun20)},
};
u8 key_flag=0;
u8 flag=0;//按键模式切换标志位
extern bool_t usart_flag;
u8 count2=0;
extern u8 res;
int change_flag=0;
unsigned char finnal_change_data=0;
extern void OLED_Task(void *pvParameters)
{	
	OLED_Clear();//清屏
	vTaskDelay(10);
	OLED_StartDisplay();
  motor_diaplay.RC_data=get_RC_DATD();
	for(int i=0;i<4;i++)
	{motor_diaplay.point_motor[i]=get_Motor_3508_Point(i);}
	while(1)
	{
		if(flag==0)
		{
	    KEY_WorkSet1();
		}
		else if(flag==1)
		{
		KEY_WorkSet2();
		}
		else if(flag==2)
		{
		KEY_WorkSet3();
		}
		else if(flag==3)
		{
		KEY_WorkSet4();
		}
		current_operation_index = table[func_index].current_operation;
    (*current_operation_index)(); //执行当前操作函数
	}
} 
vu8 key=0;
static void KEY_WorkSet1(void)
{
#if (TEST==1)	
	key=KEY_Scan(1);	//得到键值
	if(key)
		{			
	
			switch(key)
			{				 
				case KEY_UP:	//上
				{
				if(key_flag==0)	
				OLED_Clear();
				func_index = table[func_index].up;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}
				break; 
				case KEY_DOWN:	//下
				{	
      if(key_flag==0)					
				OLED_Clear();
				func_index = table[func_index].next ;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}
				break;
				case KEY_CONFRIM:	//进入
				{
				OLED_Clear();
				func_index = table[func_index].enter ;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}
				break;
			case KEY_RETURN:
			{
				OLED_Clear();
				func_index = table[func_index].Return ;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
			}
				break;
			}
		}
#elif (TEST==0)
			key=KEY_Scan(0);	//得到键值
	if(key)
		{						   
			switch(key)
			{				 
				case WKUP_PRES:	//上
				{
					
				OLED_Clear();
				func_index = table[func_index].up;
				while(!key);
				}
				break; 
				case KEY1_PRES:	//下
				{					
				OLED_Clear();
				func_index = table[func_index].next ;
				while(!key);
				}
				break;
				case KEY0_PRES:	//进入
				{
				OLED_Clear();
				func_index = table[func_index].enter ;
				while(!key);
				}
				break;
			}
		}
#endif

//	OLED_Refresh();
}
int_least8_t data_flag1=1,data_flag2=0;
static void KEY_WorkSet2(void)
{
	key=KEY_Scan(1);	//得到键值
	if(key)
		{			
	
			switch(key)
			{				 
				case KEY_UP:	//上
				{
					data_flag1++;
					data_flag2++;
					id++;
					if(id>9){id=1;}
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}
				break; 
				case KEY_DOWN:	//下
				{		
         data_flag2--;				
					data_flag1--;
					if(data_flag1<0)
					{data_flag1=0;}
					id--;
					if(id<1){id=1;}
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}
				break;
				case KEY_LEFT:	//左
				{					
					count2--;
       data_flag1=1;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}	
				break;
			case KEY_RIGHT:	//右
				{					
					count2++;
				data_flag1=1;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}	
				break;
     case KEY_CONFRIM:	//进入
				{
				func_index = table[func_index].enter ;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}
				break;
			case KEY_RETURN:
			{
				flag=0;
				id=0;
				data_flag1=1;
				data_flag2=0;
				OLED_Clear();
				func_index = table[func_index].Return ;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
			}
			}
		}	
}
static void KEY_WorkSet3(void)
{
	key=KEY_Scan(1);	//得到键值
	if(key)
		{			
	
			switch(key)
			{				 
				case KEY_UP:	//上
				{
					
				if(key_flag==1)	
				{
					motor_diaplay.display[subscript]=motor_diaplay.display[subscript]+1;
					if(motor_diaplay.display[subscript]>9)
					{
						motor_diaplay.display[subscript]=0;
						if(subscript>subscript_min)
						motor_diaplay.display[subscript-1]=motor_diaplay.display[subscript-1]+1;
					}
				}
				if(key_flag==3)
				   {
					  angle [subscript]=angle [subscript]+1;
						if(angle [subscript]>9)
						{
							angle [subscript]=0;
							if(subscript>subscript_min)
								angle [subscript-1]=angle [subscript-1]+1;
						}
					}
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}
				break; 
				case KEY_DOWN:	//下
				{					
					if(key_flag==1)
					{
					motor_diaplay.display[subscript]=motor_diaplay.display[subscript]-1;
					if(motor_diaplay.display[subscript]<0)
					{
						motor_diaplay.display[subscript]=9;
					motor_diaplay.display[subscript-1]=motor_diaplay.display[subscript-1]-1;
					}
			  	}
					if(key_flag==3)
					{
					angle [subscript]=angle [subscript]-1;
						if(angle [subscript]<0)
						{
							angle [subscript]=9;
							angle [subscript-1]=angle [subscript-1]-1;
						}
					}
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}	
				break;
			case KEY_LEFT:	//左
				{					
//					count2=0;
					subscript--;
					if(key_flag==1)
          {if(subscript<0){subscript=3;}	}	
					else if(key_flag==3)
					{if(subscript<0){subscript=2;}}
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}	
				break;
			case KEY_RIGHT:	//右
				{					
//					count2=0;
					subscript++;
					if(key_flag==1)
					{if(subscript>3)	{subscript=0;}	}
            else if(key_flag==3)	
						{if(subscript>2)	{subscript=0;}}							
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}	
				break;
     case KEY_CONFRIM:	//进入
				{
				OLED_Clear();
				func_index = table[func_index].enter ;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}
				break;
			case KEY_RETURN:
			{
				flag=0;
				subscript=0;
				OLED_Clear();
				func_index = table[func_index].Return ;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
			}
			}
		}	
}
static void KEY_WorkSet4(void)
{
	key=KEY_Scan(1);//得到键值  configTOTAL_HEAP_SIZE
	if(key)
		{			
	
			switch(key)
			{				 
				case KEY_UP:	//上
				{
					change_flag++;

					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}
				break; 
				case KEY_DOWN:	//下
				{					
            change_flag--;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}
				break;
     case KEY_CONFRIM:	//进入
				{
				func_index = table[func_index].enter ;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
				}
				break;
			case KEY_RETURN:
			{
				flag=0;
				OLED_Clear();
				func_index = table[func_index].Return ;
					while(key!=0)
					{
					key=KEY_Scan(1);
					}
			}
			}
		}	
}
static void OLED_StartDisplay(void)
{
	int n = 8;
	key_flag=0;
	vTaskDelay(100);
	OLED_ShowCHinese(3*n,1,0);//电
	OLED_ShowCHinese(5*n,1,1);//机
	OLED_ShowCHinese(7*n,1,2);//调
	OLED_ShowCHinese(9*n,1,3);//试
	OLED_ShowCHinese(11*n,1,4);//板
	OLED_ShowCHinese(4*n,4,5);//任
	OLED_ShowCHinese(6*n,4,6);//意
	OLED_ShowCHinese(8*n,4,7);//键
	OLED_ShowCHinese(10*n,4,8);//继
	OLED_ShowCHinese(12*n,4,9);//续	
	
	OLED_ShowString(12*n,6,"...",32);  
}
static void OLED_MainMenuDisplay(void)
{
	int n = 8;
	vTaskDelay(100);
	OLED_ShowString(n,0,"1",16);//1
	OLED_ShowString(n,2,"2",16);//2
	OLED_ShowString(n,4,"3",16);//3
	OLED_ShowString(n,6,"4",16);//3
	
	OLED_ShowString(2*n,1,".",24);  
	OLED_ShowString(2*n,3,".",24);  
	OLED_ShowString(2*n,5,".",24); 
  OLED_ShowString(2*n,7,".",24);	

	OLED_ShowCHinese(3*n,0,10);//电
	OLED_ShowCHinese(5*n,0,11);//机
	
	OLED_ShowCHinese(3*n,2,14);//陀
	OLED_ShowCHinese(5*n,2,15);//螺
	OLED_ShowCHinese(7*n,2,16);//仪	
	
	OLED_ShowCHinese(3*n,4,23);//遥
	OLED_ShowCHinese(5*n,4,24);//控
	OLED_ShowCHinese(7*n,4,25);//器
	
	OLED_ShowCHinese(3*n,6,39);//串
	OLED_ShowCHinese(5*n,6,40);//口
	OLED_ShowCHinese(7*n,6,41);//调
	OLED_ShowCHinese(9*n,6,42);//试
}
static void OLED_MotorMenuDisplay(void)
{
	int n = 8;
	vTaskDelay(100);
	OLED_ShowString(n,0,"1",16);//1
	OLED_ShowString(n,2,"2",16);//2
	OLED_ShowString(n,4,"3",16);//3
	OLED_ShowString(n,6,"4",16);
	
	OLED_ShowString(2*n,1,".",24);  
	OLED_ShowString(2*n,3,".",24);  
	OLED_ShowString(2*n,5,".",24);
  OLED_ShowString(2*n,7,".",24);


	
	OLED_ShowString(3*n,0,"M3508",16); 
	
	OLED_ShowString(3*n,2,"GM6020",16); 
	
	OLED_ShowString(3*n,4,"M2006",16); 
	
	OLED_ShowCHinese(3*n,6,12);//舵
	OLED_ShowCHinese(5*n,6,13);//机
}

static void OLED_MotorSetDisplay(void)//电机性能设置界面
{
	int n = 8;
	vTaskDelay(100);
	OLED_ShowString(n,0,"1",16);//1
	OLED_ShowString(n,3,"2",16);//3
	OLED_ShowString(n,6,"3",16);

	OLED_ShowString(2*n,1,".",24);  
	OLED_ShowString(2*n,4,".",24);
	OLED_ShowString(2*n,7,".",24);
	
	OLED_ShowString(3*n,0,"ID",16); 
	OLED_ShowString(3*n,3,"SET",16);
  OLED_ShowString(3*n,6,"FPB",16);	
	
	OLED_ShowString(6*n,0,":",16); 
	OLED_ShowString(6*n,3,":",16);
	OLED_ShowString(6*n,6,":",16);


	OLED_ShowNum(7*n,0,id,1,16);//
	for(int i=0;i<4;i++)
	{ 
		  OLED_ShowNum((7+i)*n,3,motor_diaplay.display[i],1,16);
	  	OLED_ShowNum((7+i)*n,6,motor_diaplay.fpb_display[i],1,16);
	}

	
}

static void OLED_MotorFeedbackDisplay(void)//电机反馈结果界面
{
	int n = 8;
	vTaskDelay(100);
	OLED_ShowString(n,0,"1",16);//1
	OLED_ShowString(n,3,"2",16);//3
	OLED_ShowString(n,6,"3",16);

	OLED_ShowString(2*n,1,".",24);  
	OLED_ShowString(2*n,4,".",24);
	OLED_ShowString(2*n,7,".",24);
	
	OLED_ShowString(3*n,0,"ID",16); 
	OLED_ShowString(3*n,3,"SET",16);
  OLED_ShowString(3*n,6,"FPB",16);	
	
	OLED_ShowString(6*n,0,":",16); 
	OLED_ShowString(6*n,3,":",16);
	OLED_ShowString(6*n,6,":",16);


	OLED_ShowNum(7*n,0,id,1,16);//
	for(int i=0;i<4;i++)
	{ 
		  OLED_ShowNum((7+i)*n,3,motor_diaplay.display[i],1,16);
	  	OLED_ShowNum((7+i)*n,6,motor_diaplay.fpb_display[i],1,16);
	}

}
void Steer_engine_display()
{
   int n=8;
  vTaskDelay(100);
  OLED_ShowString(n,1,"1",16);//1
	OLED_ShowString(n,3,"2",16);//3
	OLED_ShowString(n,5,"3",16);//3
	
	OLED_ShowString(2*n,2,".",6);  
	OLED_ShowString(2*n,4,".",6);
	OLED_ShowString(2*n,6,".",6);
	
  OLED_ShowCHinese(3*n,1,31);//模
  OLED_ShowCHinese(5*n,1,32);//拟
	OLED_ShowCHinese(7*n,1,35);//舵
  OLED_ShowCHinese(9*n,1,36);//机
	
  OLED_ShowCHinese(3*n,3,33);//数
  OLED_ShowCHinese(5*n,3,34);//字
	OLED_ShowCHinese(7*n,3,35);//舵
  OLED_ShowCHinese(9*n,3,36);//机	
	
  OLED_ShowCHinese(3*n,5,37);//数
  OLED_ShowCHinese(5*n,5,38);//字	

}
void angle_set()
{
	int n=8;
  vTaskDelay(100);
	OLED_ShowString(n,0,"angle:",16);//1

	for(int i=0;i<3;i++)
	{
  OLED_ShowNum((7+i)*n,0,angle[i],1,16);
  }
}
char DATAA=0;
void angle_display()
{
	int n=8;
  vTaskDelay(100);
	OLED_ShowString(n,0,"angle:",16);//1

	for(int i=0;i<3;i++)
	{
  OLED_ShowNum((7+i)*n,0,angle[i],1,16);
  }
	for(int j=2;j<5;j++)
	{
		if(j==3)
		{
			for(int i=2;i<100;i++)
			{
				if((i>=20+Angle/6)&&(i<=80))
				{
				if((i==(Angle/6+20))||(i==80))
					DATAA=0xFE;
				else DATAA=02;
			  }
				else DATAA=0x00;
#if (Reverse_display==1)
     				finnal_change_data=transform_data(DATAA);
							OLED_Set_Pos(127-i,7-j);		
						 OLED_WR_Byte(finnal_change_data,OLED_DATA);
#elif (Reverse_display==0)
             OLED_Set_Pos(i,j);
				OLED_WR_Byte(DATAA,OLED_DATA);
				
#endif
			}
		}
		else if(j==4)
		{
					for(int i=2;i<100;i++)
			{
				if((i>=20+Angle/6)&&(i<=80))
				{
					if((i==(Angle/6+20))||(i==80))
					DATAA=0x7F;
					else DATAA=0x00;
				}
				else DATAA=0x40;
#if (Reverse_display==1)
			   	finnal_change_data=transform_data(DATAA);
							OLED_Set_Pos(127-i,7-j);
						 OLED_WR_Byte(finnal_change_data,OLED_DATA);
#elif (Reverse_display==0)
             OLED_Set_Pos(i,j);
				OLED_WR_Byte(DATAA,OLED_DATA);
				
#endif
			}
		
		}
//		else if(j==4)
//		{
//			OLED_Set_Pos(16,j);
//		 for(int i=0;i<11;i++)
//			{
//			OLED_WR_Byte(steer_motor[0][i],OLED_DATA);//0
//			}
//		 	OLED_Set_Pos(30,j);
//		 for(int i=0;i<11;i++)
//			{
//			OLED_WR_Byte(steer_motor[1][i],OLED_DATA);//0.5
//			}			
//			OLED_Set_Pos(45,j);
//		 for(int i=0;i<11;i++)
//			{
//			OLED_WR_Byte(steer_motor[2][i],OLED_DATA);//1.0
//			}			
//			OLED_Set_Pos(60,j);
//		 for(int i=0;i<11;i++)
//			{
//			OLED_WR_Byte(steer_motor[3][i],OLED_DATA);//1.5
//			}
//			OLED_Set_Pos(75,j);
//		 for(int i=0;i<11;i++)
//			{
//			OLED_WR_Byte(steer_motor[4][i],OLED_DATA);//2.0
//			}		
//		}
}
//	OLED_ShowString(92,4,"(ms)",6);
}
u32 usrt_data=115200;
char usrt_dis_flag;
char usart_point=0x01,stop_bit=1,data_bit=8;
char data_judge[6]={0};
u8 dexi=0,dex_x=0,sine;
void usart_display()
{
	
	   vTaskDelay(100);
    char ch[2]	;
    sprintf(ch,"%x",res);

	  OLED_ShowNum(0,1,usrt_data,6,6);//波特率  baud rate
	  OLED_ShowString(48,1,"/",6);//
	  OLED_ShowNum(56,1,stop_bit,1,6);//停止位  stop bit
	  OLED_ShowString(64,1,"/",6);//
		OLED_ShowNum(72,1,data_bit,1,6);//数据位  data bit
	  OLED_ShowString(80,1,"/",6);//
		OLED_ShowString(88,1,"N",6);//校验位  check bit
	  OLED_ShowString(64,1,"/",6);//
	  OLED_ShowString(5,4,"RX",6);
	  OLED_ShowString(5,7,"Tx",6);
	  OLED_ShowString(60,7,"00",6);
	  OLED_ShowString(100,7,"aa",6);

if(usrt_dis_flag==1)
			OLED_ShowString(106,1,"*",6);
else if(usrt_dis_flag==2)
{
	if(count2%2==0)
	{sine=53;}
	else if(count2%2==1)
		{sine=93;}
	OLED_ShowString(sine,7,"*",6);
}
	if(usart_flag==1)
	{
		usart_flag=0;
	dexi+=16;
		if(dexi>64)
		{
		dexi=0;
			dex_x+=1;
			if(dex_x==3)
				dex_x=0;
		}
	}
	  OLED_ShowString(45+dexi,3+dex_x,ch,6);
  for(int j=2;j<8;j++)
	{
		if(j==2||j==6)
		{
		for(int i=0;i<128;i++)
			{
				if(i==42&&j==2)
				{usart_point=0xf8;}
				else if (i==42&&j==6)
				{usart_point=0xff;}
				else usart_point=0x08;
#if (Reverse_display==1)
					OLED_Set_Pos(127-i,7-j);
				finnal_change_data=transform_data(usart_point);
				 OLED_WR_Byte(finnal_change_data,OLED_DATA);
#elif (Reverse_display==0)
				OLED_Set_Pos(i,j);
				OLED_WR_Byte(usart_point,OLED_DATA);
#endif
			}
		}
    else 
		{
		
			usart_point=0xff;
			
#if (Reverse_display==1)			
			OLED_Set_Pos(127-42,7-j);
			finnal_change_data=transform_data(usart_point);
			OLED_WR_Byte(finnal_change_data,OLED_DATA);
#elif (Reverse_display==0)	
				OLED_Set_Pos(42,j);
				OLED_WR_Byte(usart_point,OLED_DATA);
#endif			
		}
		
	
	}
}

void  mcufeedbackDisplay(float temperature,float yaw,float pitch,float roll)
{
//	int n = 8;
	vTaskDelay(100);

	char_display(2,0,0,3,&mpu[0][0]);//TEM
	char_display(2,2,3,3,&mpu[3][0]);//YAW
	char_display(2,4,6,3,&mpu[6][0]);//PIT
	char_display(2,6,9,4,&mpu[9][0]);//ROLL

//:	
	char_display(35,0,13,1,&mpu[13][0]); 
	char_display(35,2,13,1,&mpu[13][0]); 
	char_display(35,4,13,1,&mpu[13][0]); 
	char_display(35,6,13,1,&mpu[13][0]); 
//反馈值		
	if(temperature<0)
	{
	
	   if(temperature<=-100)
			{
			OLED_ShowString(42,0,"-",16);
			OLED_ShowNum(50,0,temperature,3,16);
			OLED_ShowCHinese(74,0,28);//℃
			}
			else if ((temperature>-100)&&(temperature<=-10))
			{
			OLED_ShowString(42,0,"-",16);
			OLED_ShowNum(50,0,temperature,2,16);
			OLED_ShowCHinese(66,0,28);//℃
				OLED_ShowString(82,0," ",16);
			}
			else if(temperature>-10)
			{
			OLED_ShowString(50,0,"-",16);
			OLED_ShowNum(58,0,temperature,1,16);
			OLED_ShowCHinese(66,0,28);//℃
				OLED_ShowString(82,0," ",16);
			}
				temperature=-temperature;
	}
	else 
	{
	  OLED_ShowNum(42,0,temperature,3,16);
		OLED_ShowCHinese(66,0,28);
		OLED_ShowString(82,0," ",16);
	}
		if(yaw<0)
	{
		
	   if(yaw<=-100)
			{
				OLED_ShowString(42,2,"-",16);
				OLED_ShowNum(50,2,yaw,3,16);
				OLED_ShowCHinese(74,2,29);//°
			
			}
			else if ((yaw>-100)&&(yaw<=-10))
			{
			OLED_ShowString(42,2,"-",16);
			OLED_ShowNum(50,2,yaw,2,16);
			OLED_ShowCHinese(66,2,29);//℃
			OLED_ShowString(82,2," ",16);
			}
			else if(yaw>-10)
			{
			OLED_ShowString(50,2,"-",16);
			OLED_ShowNum(58,2,yaw,1,16);
			OLED_ShowCHinese(66,2,29);//℃
			OLED_ShowString(82,2," ",16);
			}
			yaw=-yaw;
	}
	else
	{
	  OLED_ShowNum(42,2,yaw,3,16);
		OLED_ShowCHinese(66,2,29);
		OLED_ShowString(82,2," ",16);
	}		
	if(pitch<0)
	{
	
	   if(pitch<=-100)
			{
				OLED_ShowString(42,4,"-",16);
				OLED_ShowNum(50,4,pitch,3,16);
				OLED_ShowCHinese(74,4,29);//°
			
			}
			else if ((pitch>-100)&&(pitch<=-10))
			{
			OLED_ShowString(42,4,"-",16);
			OLED_ShowNum(50,4,pitch,2,16);
			OLED_ShowCHinese(66,4,29);//℃
			OLED_ShowString(82,4," ",16);
			}
			else if(pitch>-10)
			{
			OLED_ShowString(50,4,"-",16);
			OLED_ShowNum(58,4,pitch,1,16);
			OLED_ShowCHinese(66,4,29);//℃
			OLED_ShowString(82,4," ",16);
			}
				pitch=-pitch;
	}
	else
	{
	OLED_ShowCHinese(66,4,29);//°
	OLED_ShowNum(42,4,pitch,3,16);
	OLED_ShowString(82,4," ",16);
	}
	if(roll<0)
	{
		
		 if(roll<=-100)
			{
				OLED_ShowString(42,6,"-",16);
				OLED_ShowNum(50,6,roll,3,16);
				OLED_ShowCHinese(74,6,29);//°
				
			
			}
			else if ((roll>-100)&&(roll<=-10))
			{
			OLED_ShowString(42,6,"-",16);
			OLED_ShowNum(50,6,roll,2,16);
			OLED_ShowCHinese(66,6,29);//℃
			OLED_ShowString(82,6," ",16);
			}
			else if(roll>-10)
			{
			OLED_ShowString(50,6,"-",16);
			OLED_ShowNum(58,6,roll,1,16);
			OLED_ShowCHinese(66,6,29);//℃
			OLED_ShowString(82,6," ",16);
			}
		roll=-roll;
	}
	else 
	{
	OLED_ShowNum(42,6,roll,3,16);
	OLED_ShowCHinese(66,6,29);//°
		OLED_ShowString(82,6," ",16);
	}
	
}
char point=0x01,point0=0x81;
int Remote_ch[5],Percentage,Whole_lattive,Semilattice,Remote_S[2],Remote_per[5];
void RomoteFeedbackDisplay()
{
// int n=8;
	vTaskDelay(100);
//显示Remote task
	char_display(20,0,0,11,&ceshi[0][0]);

//通道界面显示
	for(int j=2;j<8;j++)
	{

		 if(j==7)
		{

			if(change_flag%2==0)
				OLED_ShowString(16,j,"D",8);
			else if(change_flag%2==1)
				OLED_ShowString(16,j,"%",8);
					for(int i=25;i<128;i++)
				{
					if(i==127||i==INIT_DATA||i==(INIT_DATA+INTERVAL_DATA)||i==(INTERVAL_DATA*2+INIT_DATA)||i==(INTERVAL_DATA*3+INIT_DATA)||i==76||
						         i==INIT2_DATA||i==(INIT2_DATA+INTERVAL_DATA)||i==(INTERVAL_DATA*2+INIT2_DATA)||i==(INTERVAL_DATA*3+INIT2_DATA))point0=0xff;//画分界线  draw the dividing line
					else {point0=0x81;}
//s1				
          if(i==25||i==127)	point0=0x6D;				
					if((motor_diaplay.RC_data->rc.s[1]==3)&&((i<INTERVAL_DATA*2+INIT_DATA)&&(i>INIT_DATA+INTERVAL_DATA)))
					{
					  point0=0xff;
					}	
					if((motor_diaplay.RC_data->rc.s[1]==1)&&((i<INTERVAL_DATA+INIT_DATA)&&(i>INIT_DATA)))	
					{
					point0=0xff;
					}
					if((motor_diaplay.RC_data->rc.s[1]==2)&&((i<INTERVAL_DATA*3+INIT_DATA)&&(i>INIT_DATA+INTERVAL_DATA*2)))	
					{
					point0=0xff;
					}	
//s2					
					if((motor_diaplay.RC_data->rc.s[0]==3)&&((i<INTERVAL_DATA*2+INIT2_DATA)&&(i>INIT2_DATA+INTERVAL_DATA)))//
					{
					  point0=0xff;
					}	
					if((motor_diaplay.RC_data->rc.s[0]==1)&&((i<INTERVAL_DATA+INIT2_DATA)&&(i>INIT2_DATA)))	//
					{
					point0=0xff;
					}
					if((motor_diaplay.RC_data->rc.s[0]==2)&&((i<INTERVAL_DATA*3+INIT2_DATA)&&(i>INIT2_DATA+INTERVAL_DATA*2)))	
					{
					point0=0xff;
					}	
//          data_Ch[j][i]=point0;	
#if (Reverse_display==1)						
					OLED_Set_Pos(127-i,7-j);
					finnal_change_data=transform_data(point0);
				 OLED_WR_Byte(finnal_change_data,OLED_DATA);
#elif (Reverse_display==0)
          OLED_Set_Pos(i,j);
					OLED_WR_Byte(point0,OLED_DATA);
#endif					
				}
		}
		 else
		{
      			
			  Percentage=motor_diaplay.RC_data->rc.ch[j-2]/6.6;//遥控器最大值为660 每6.6为1%   the maximum value of the remote controller is 660,which is 1% every 6.6
      	Whole_lattive=Percentage/2;//每一小格代表两个百分点    each small box represents two percentage points
	      Semilattice=Percentage%2;//判断百分数值是否为奇数     judge whether the percentage is odd

			if(change_flag%2==0)
			{OLED_ShowNum(0,j,abs(motor_diaplay.RC_data->rc.ch[j-2]),3,6);}	
			else if(change_flag%2==1)
			OLED_ShowNum(0,j,abs(Percentage),3,6);

				for(int i=25;i<128;i++)
				{
					if(i==MID_DATA)point=0xff;
					else if((i-25)%5==0)
					{point=0x00;}
						else point=0x01;
					if(i==127||i==25)
						point=0x6D;
					if(Percentage!=0)
					{
						if(motor_diaplay.RC_data->rc.ch[j-2]>0)//数值为正往右显示
						{
							if((i<=MID_DATA+Whole_lattive)&&(i>MID_DATA))
							{
								if((i-25)%5==0)point=0xFE;
							else point=0xFF;
								
								
							}
							if((Semilattice!=0)&&(i==MID_DATA+Whole_lattive+1))//如果百分比为奇数 最后一格显示半格
							{
								point=0xf1;
							}
						}
						else if(motor_diaplay.RC_data->rc.ch[j-2]<0)//数值为负往左显示
						{
							if((i>=MID_DATA+Whole_lattive)&&(i<MID_DATA))
							{
								if((i-25)%5==0)point=0xFE;
								else point=0xFF;
								
							}
							if((Semilattice!=0)&&(i==MID_DATA+Whole_lattive-1))
							{
                 point=0xf1;
							}
						}
					}
//					data_Ch[j][i]=point;
#if (Reverse_display==1)		
					finnal_change_data=transform_data(point);
				 OLED_Set_Pos(127-i,7-j);//设置坐标
				 OLED_WR_Byte(finnal_change_data,OLED_DATA);//显示内容
#elif (Reverse_display==0)
         OLED_Set_Pos(i,j);//设置坐标
				 OLED_WR_Byte(point,OLED_DATA);//显示内容
#endif					
				}
		}
  }
//	change_data(25,10,(char *)ceshi);
//	for(int i=0;i<8;i++)
//	{
//	    for(int)
//	}
//	OLED_ShowString(28, 7, "s1", 8);
}
void fun0(void)//电机
{
	int n = 8;
	key_flag=1;
	OLED_MainMenuDisplay();
	vTaskDelay(100);
	OLED_ShowString(n,0," ", 16);//
}

void fun1(void)//陀螺仪
{
	int n = 8;
	OLED_MainMenuDisplay();
	vTaskDelay(100);
	OLED_ShowString(n, 2, "  ", 16);
}
void fun2(void)//遥控器
{
	int n = 8;

	OLED_MainMenuDisplay();
	vTaskDelay(100);
	OLED_ShowString(n, 4, "  ", 16);
}
void fun4(void)//串口调试
{
	int n = 8;

	OLED_MainMenuDisplay();
	vTaskDelay(100);
	OLED_ShowString(n, 6, "  ", 16);
}
void fun5(void)//舵机
{
	int n = 8;
		Motor_Set=0;
	motor_diaplay.display[0]=0;motor_diaplay.display[1]=0;motor_diaplay.display[2]=0;motor_diaplay.display[3]=0;
	OLED_MotorMenuDisplay();
	vTaskDelay(100);
	OLED_ShowString(n, 0, "  ", 16);
}	
void fun6(void)//3508
{
	int n = 8;
		id=1;
	OLED_MotorMenuDisplay();
	vTaskDelay(100);
	OLED_ShowString(n, 2, "  ", 16);
}
void fun7(void)//6020
{
	int n = 8;
	OLED_MotorMenuDisplay();
	vTaskDelay(100);
	OLED_ShowString(n, 4, "  ", 16);
}
void fun8(void)//2006
{
	int n = 8;
	OLED_MotorMenuDisplay();
	vTaskDelay(100);
	OLED_ShowString(n, 6, "  ", 16);
}
//电调id
void fun9(void)
{
	flag=1;
	
	OLED_MotorSetDisplay();
}
//设置转速
void fun10(void)
{
		int n = 8;
  flag=2;
	key_flag=1;
	OLED_MotorSetDisplay();	
	vTaskDelay(100);
	OLED_ShowString((7+subscript)*n,3," ",16);
}

void fun11(void)
{
	key_flag=2;
  Motor_Set=motor_diaplay.display [0]*1000+motor_diaplay.display [1]*100+motor_diaplay.display [2]*10+motor_diaplay.display [3];
	Motor_play(motor_diaplay.point_motor[id-1]->speed_rpm);
	OLED_MotorFeedbackDisplay();
}
//遥控器显示
void fun12(void)
{
   flag=3;
	RomoteFeedbackDisplay();
}
//陀螺仪显示
void fun13(void)
{
  flag=3;
	steer_flag=1;
  mcufeedbackDisplay(mpu_data_feed.temperature,mpu_data_feed.yaw,mpu_data_feed.pitch,mpu_data_feed.roll);
}
void fun14(void)
{
	int n=8;
 flag=0;
	steer_flag=2;
  Steer_engine_display();
	vTaskDelay(100);
	OLED_ShowString(n,1," ",16);
}
void fun15(void)
{
	int n=8;
 flag=0;
  Steer_engine_display();
	vTaskDelay(100);
	OLED_ShowString(n,3," ",16);
}
void fun16(void)
{
	int n=8;
 flag=0;
  Steer_engine_display();
	vTaskDelay(100);
	OLED_ShowString(n,5," ",16);
}
void fun17(void)
{
	u8 n=8;
	key_flag=3;
	flag=2;
			if(angle[0]>3)
	{angle[0]=3;}
	if(angle[0]==3)
	{
	if(angle[1]>=6)
	{angle[1]=6;angle[2]=0;}
	}
  angle_set();
	vTaskDelay(100);
	OLED_ShowString((7+subscript)*n,0," ",16);
}
void fun18(void)
{
//	u8 n=8;
	key_flag=3;
	flag=2;
			if(angle[0]>3)
	{angle[0]=3;}
	if(angle[0]==3)
	{
	if(angle[1]>=6)
	{angle[1]=6;angle[2]=0;}
	}
	Angle=angle[0]*100+angle[1]*10+angle[2];
  angle_display();
//	vTaskDelay(100);
}

void fun19(void)
{
	flag=1;
	usrt_dis_flag=1;
	if(count2%3==0)
	{
			if(id%3==0)
			{
			usrt_data=9600;
			}
			else if(id%3==1)
			{
			usrt_data=115200;
			}
			else if(id%3==2)
			{
			usrt_data=921600;
			}
	}
	else if(count2%3==1)
	{
	   stop_bit= data_flag1%4;
				
	}
  else if(count2%3==2)
	{
	  if(data_flag2%2==0)
			data_bit=8;
		else data_bit=9;
	}
  usart_display();
	vTaskDelay(100);
  OLED_ShowString(106,1," ",6);

}
void fun20(void)
{
  flag=1;
	usrt_dis_flag=2;
  usart_display();
	if(sine==53)
		USART_SendData(USART3,0x00);
	else USART_SendData(USART3,0xaa);
	vTaskDelay(100);
  OLED_ShowString(sine,7," ",6);

}
void Motor_play(int16_t data_play)
{

  motor_diaplay.fpb_display[0]=data_play/1000;
  motor_diaplay.fpb_display[1]=data_play%1000/100;
	motor_diaplay.fpb_display[2]=data_play%100/10;
  motor_diaplay.fpb_display[3]=data_play%10;

}
void char_display(u8 x,u8 y,u8 deviation,u8 length,char *data)
{
	for(int j=0+deviation;j<deviation+length;j++)
	{
      	
				for(int i=0;i<8;i++)
				{
#if (Reverse_display==1)
					    finnal_change_data=transform_data(*data);
							OLED_Set_Pos(127-(x+(j-deviation)*8)-i,7-y);
							 OLED_WR_Byte(finnal_change_data,OLED_DATA);
#elif  (Reverse_display==0)
							OLED_Set_Pos((x+(j-deviation)*8)+i,y);
				OLED_WR_Byte(*data,OLED_DATA);		
#endif					
					data++;
				}
				
					for(int i=0;i<8;i++)
				{
#if (Reverse_display==1)
					finnal_change_data=transform_data(*data);
					   	OLED_Set_Pos(127-(x+(j-deviation)*8)-i,7-(y+1));
							 OLED_WR_Byte(finnal_change_data,OLED_DATA);
#elif  (Reverse_display==0)
								OLED_Set_Pos((x+(j-deviation)*8)+i,y+1);
			       	OLED_WR_Byte(*data,OLED_DATA);		
#endif
					data++;
				}
}

}
//void change_data(int x,int y,char *data)
//{
//  for(int j=0+y/8;j<8;j++)
//	{
//	    for(int i=0+x;i<128;i++)
//		{
//		   for(int n=0;n<8;n++)
//			{
//			   Change_data[n+j*8+y%8][i]=(*data>>n)&0x01;
//			}
//		data++;
//		}
//	
//	}
//	
//	for(int j=0+y/8;j<8;j++)
//	{
//	   for(int i=0+x;i<128;i++)
//		{
//		    for(int n=0;n<8;n++)
//			{
//			   data_pre[j][i]=data_pre[j][i]|(Change_data[n+j*8][i]<<n);
//			}
//						 OLED_Set_Pos(i,j);//设置坐标
//				 OLED_WR_Byte(data_pre[j][i],OLED_DATA);//显示内容
//		}
//	
//	
//	}


//}

