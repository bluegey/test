#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "gpio.h"
#include "timer.h"
#include "tcs34725.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


extern u8 a ;
extern int32_t time1_cntr;
//extern COLOR_RGBC rgb;
//extern COLOR_HSL  hsl;
//extern COLOR_RGBC rgb_1;
//extern COLOR_HSL  hsl_1;
//extern COLOR_RGBC rgb_2;
//extern COLOR_HSL  hsl_2;
//extern COLOR_RGBC rgb_3;
//extern COLOR_HSL  hsl_3;
//extern COLOR_RGBC rgb_4;
//extern COLOR_HSL  hsl_4;
int PA15, medicine_flag = 1, cnt_flg = 1, remember_num, rem_flg = 1, scan_flag = 1,scan_ture;
float yaw0;
//int32_t last_hsl_h, last_hsl1_h, last_hsl2_h, last_hsl3_h, last_hsl4_h;
/********************************转弯****************************/
void turn_left(int x);
void turn_right(int x);
void guizhong(void);
/********************************跟随/停止****************************/
void Follow_mpu(int fangxiang, int turn_ange);
void stop(void);
/********************************前进/后退****************************/
void go_forward(int x2, int x3);
void go_back(int x1, int x2);
/********************************行动函数****************************/
void goto_1(void);
void goto_2(void);
void goto_others(void);
/********************************摄像头扫描函数****************************/
void scan_left(int x);
void scan_right(int x);
void scan_mid(void);

void rem_num(void);

void delay_s(u32 i)
{
    while(i--);
}

int main(void)
{
    gpio_init();//IO口初始化
    delay_init(168);
    MPU_Init();					//初始化MPU6050

    while(mpu_dmp_init());

    delay_init(168);
    uart_init(115200);

    TIM5_PWM_Init(1000 - 1, 1680 - 1);//50Hz
    TIM3_PWM_Init(10 - 1, 84 - 1);//100KHz
    TIM4_PWM_Init(10 - 1, 84 - 1);//100KHz
    TIM1_Int_Init(1000 - 1, 84 - 1);//1000KHz			1ms
    guizhong();
    scan_mid();
    while(1)
    {
        mpu_dmp_get_yaw(&yaw0);//获取陀螺仪YAW值
        PA15 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10);//检测重物是否放下 1/0

        if(PA15 == 0 && medicine_flag == 1) //停止不动
        {
            stop();
        }

        goto_1();
        goto_2();
        goto_others();
        /**************************************************************************/

    }
}

void goto_1(void)
{
    if(PA15 == 1 && a == 1) //装载后		识别到数字		开始计时
    {
        medicine_flag = 2;

        if(cnt_flg == 1)
        {
            cnt_flg = 2;
            TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //开始计时
        }

        if(time1_cntr >= 0 && time1_cntr < 5500)
        {
            Follow_mpu(1, 0); //走直线
        }
        else if(time1_cntr >= 5500 && time1_cntr < 9800)
        {
            turn_left(80);//左打舵
            go_forward(10, 6);
        }
        else if(time1_cntr >= 9800 && time1_cntr < 14000)
        {
            guizhong();
            Follow_mpu(1, 90); //走直线
        }
        else if(time1_cntr >= 14000)
        {
            time1_cntr = 14000;
            stop();
            RED_ON;
        }
    }
    /**************************************************************************/
    else if(PA15 == 0 && medicine_flag == 2)//重物取走
    {
        delay_ms(1000);
        RED_OFF;
        cnt_flg = 3;

        if(time1_cntr >= 9800 && time1_cntr < 14000)
        {
            guizhong();
            Follow_mpu(0, 90); //走直线
        }
        else if(time1_cntr >= 5500 && time1_cntr < 9800)
        {
            turn_right(60);//右打舵
            go_back(6, 10);
        }
        else if(time1_cntr < 5500 && time1_cntr >= 0)
        {
            guizhong();
            Follow_mpu(1, 0); //走直线
        }
        else if(time1_cntr < 0)
        {
            GREEN_ON;
            stop();
        }
    }
}

void goto_2(void)
{
    if(PA15 == 1 && a == 2) //装载后		识别到数字		开始计时
    {
        medicine_flag = 3;

        if(cnt_flg == 1)
        {
            cnt_flg = 2;
            TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //开始计时
        }

        if(time1_cntr < 5500 && time1_cntr >= 0)
        {
            Follow_mpu(1, 0); //走直线
        }
        else if(time1_cntr >= 5500 && time1_cntr < 9800)
        {
            turn_right(60);//左打舵
            go_forward(6, 10);
        }
        else if(time1_cntr >= 9800 && time1_cntr < 14000)
        {
            guizhong();
            Follow_mpu(1, -90); //走直线
        }
        else if(time1_cntr >= 14000)
        {
            time1_cntr = 14000;
            stop();
            RED_ON;
        }
    }
    /**************************************************************************/
    else if(PA15 == 0 && medicine_flag == 3)//重物取走
    {
        delay_ms(1000);
        RED_OFF;
        cnt_flg = 3;

        if(time1_cntr >= 9800 && time1_cntr < 14000)
        {
            guizhong();
            Follow_mpu(0, -90); //走直线
        }
        else if(time1_cntr >= 5500 && time1_cntr < 9800)
        {
            turn_left(80);//左打舵
            go_back(10, 6);
        }
        else if(time1_cntr < 5500 && time1_cntr >= 0)
        {
            guizhong();
            Follow_mpu(1, 0); //走直线
        }
        else if(time1_cntr < 0)
        {
            GREEN_ON;
            stop();
        }
    }
}

void goto_others(void)
{
    if(PA15 == 1 && a != 2 && a != 1) //装载后		识别到数字		开始计时
    {
        rem_num();
        medicine_flag = 4;

        if(cnt_flg == 1)
        {
            cnt_flg = 2;
            TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //开始计时
        }

        if(time1_cntr >= 0 && time1_cntr < 8000)//前进
        {
            go_forward(6, 6);
        }
        else if(time1_cntr >= 8000 && time1_cntr < 9000)//识别
        {
            stop();

            if(scan_flag == 1)//左扫瞄
            {
                scan_left(75);
                if(a == remember_num)
                {
										scan_ture = 1;
                }
                else
                {
                    scan_flag = 2;
                }
            }
            else if(scan_flag == 2)//右扫瞄 
            {
                scan_right(65);
                if(a == remember_num)
                {
                   scan_ture = 1;
                }
                else
                {
                    scan_flag  = 0;
                }
            }
        }
				else if(time1_cntr >= 9000 && time1_cntr < 12000)//转弯
				{
					if(scan_flag == 1 && scan_ture ==1)
					{
						turn_left(80);//左打舵
            go_forward(10, 6);
					}
						else if(scan_flag == 2 && scan_ture ==1)
					{
						go_forward(6,6);
						turn_right(60);//右打舵			
            go_forward(6, 10);					
					}
				}
				else if(time1_cntr >= 12000 && time1_cntr < 15000)
				{
					if(scan_flag == 1 && scan_ture ==1)
					{
						guizhong();
						Follow_mpu(1,90);
					}
					else if(scan_flag == 2 && scan_ture ==1)
					{
						guizhong();
						Follow_mpu(1,-90);						
					}
				}
    }
}

void Follow_mpu(int fangxiang, int turn_ange)
{
    if(fangxiang == 1)
    {
        go_forward(6, 6);
    }
    else
    {
        go_back(6, 6);
    }

    if(yaw0 <  turn_ange)
    {
        if(fangxiang == 1)
        {
            TIM_SetCompare1(TIM5, 73); //左打舵
        }
        else
        {
            TIM_SetCompare1(TIM5, 67);
        }
    }
    else if(yaw0 >  turn_ange)
    {
        if(fangxiang == 1)
        {
            TIM_SetCompare1(TIM5, 67); //右打舵
        }
        else
        {
            TIM_SetCompare1(TIM5, 73); //左打舵
        }
    }
    else
    {
        TIM_SetCompare1(TIM5, 70);
    }
}
void scan_left(int x)
{
    TIM_SetCompare2(TIM5, x);
}
void scan_right(int x)
{
    TIM_SetCompare2(TIM5, x);
}
void scan_mid(void)
{
    TIM_SetCompare2(TIM5, 70);
}
void turn_left(int x)
{
    TIM_SetCompare1(TIM5, x);
}
void turn_right(int x)
{
    TIM_SetCompare1(TIM5, x);
}
void guizhong(void)
{
    TIM_SetCompare1(TIM5, 70);
}
void go_forward(int x1, int x2)
{
    GPIO_SetBits(GPIOC, GPIO_Pin_0); //左电机正转___AIN2
    GPIO_ResetBits(GPIOC, GPIO_Pin_1);//___AIN1

    GPIO_SetBits(GPIOC, GPIO_Pin_2); //右电机反转__BIN2
    GPIO_ResetBits(GPIOC, GPIO_Pin_3);//__BIN1

    TIM_SetCompare1(TIM3, x1);
    TIM_SetCompare1(TIM4, x2);
}
void go_back(int x1, int x2)
{
    GPIO_ResetBits(GPIOC, GPIO_Pin_0); //左电机反转
    GPIO_SetBits(GPIOC, GPIO_Pin_1);

    GPIO_ResetBits(GPIOC, GPIO_Pin_2); //右电机正转
    GPIO_SetBits(GPIOC, GPIO_Pin_3);

    TIM_SetCompare1(TIM3, x1);
    TIM_SetCompare1(TIM4, x2);
}
void stop(void)
{
    TIM_SetCompare1(TIM3, 10);
    TIM_SetCompare1(TIM4, 10);
}

void rem_num(void)
{
    if(rem_flg == 1)
    {
        rem_flg = 0;
        remember_num = a;
    }
}
