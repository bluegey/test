#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "gpio.h"
#include "timer.h"
#include "tcs34725.h"
#include "MPU6050.h"
extern u8 a ;
extern u32 time1_cntr;
extern COLOR_RGBC rgb;
extern COLOR_HSL  hsl;
extern COLOR_RGBC rgb_1;
extern COLOR_HSL  hsl_1;
extern COLOR_RGBC rgb_2;
extern COLOR_HSL  hsl_2;
extern COLOR_RGBC rgb_3;
extern COLOR_HSL  hsl_3;
extern COLOR_RGBC rgb_4;
extern COLOR_HSL  hsl_4;
int16_t remember_num;
int x, y, z, run_flag = 0, x1 = 8, cnt, turn_left_flag, turn_right_flag , hsl_cnt,PA15;
int32_t last_hsl_h, last_hsl1_h, last_hsl2_h, last_hsl3_h, last_hsl4_h;
void Follow(void);
void turn_left(void);
void turn_right(void);
void go_forward(void);
void go_back(void);
void stop(void);
void guizhong(void);
void remember(void);
	u16 acc_x,acc_y,acc_z,gy_x,gy_y,gy_z;	
void delay_s(u32 i)
{
    while(i--);
}
u8 medicine_flag=1;
int main(void)
{
    TCS34725_Init();//��ɫ��������ʼ��
    TCS34725_Init_1();
    TCS34725_Init_2();
    TCS34725_Init_3();
    TCS34725_Init_4();
    delay_init(168);
    uart_init(115200);
    gpio_init();//IO�ڳ�ʼ��
    TIM5_PWM_Init(1000 - 1, 1680 - 1);//84M/8400/200=50hz. �˴���200���Ǽ������ıȽ�ֵ e.g.175/200*20ms=17.5ms    �������ȣ�20ms-17.5ms=2.5ms
    TIM3_PWM_Init(10 - 1, 84 - 1);//60KHz
    TIM4_PWM_Init(10 - 1, 84 - 1);
//			TIM3_PWM_Init(200 - 1, 8400 - 1);
    TIM1_Int_Init(10 - 1, 8400 - 1);
    guizhong();

    while(1)
    {
			PA15 = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10);
			if(PA15 == 0&&medicine_flag==1)//ֹͣ����
			{
		   	TIM_SetCompare1(TIM3, 140);
        TIM_SetCompare1(TIM4, 140);
			}
			else if(PA15 ==1&&a!=0)
				{
        Follow();
					medicine_flag=2;
			  }
			else if(PA15 == 0&&medicine_flag==1)
				{
				  go_back();
				}
    }
}

void remember(void)
{
    if(	remember_num == 1 || remember_num == 2 || remember_num == 3 ||
            remember_num == 4 || remember_num == 5 || remember_num == 6 ||
            remember_num == 7 || remember_num == 8 )
    {
        run_flag = 1;
    }
    else run_flag = 0;
}

//ѭ������
void Follow(void)
{
	go_forward();
    if(time1_cntr >= 100) //1s
    {
        time1_cntr = 0;
        TCS34725_GetRawData(&rgb);  //�����Σ�ʵ�ʲ���ʱ���ֶ�������ɫ������һ�ε���ɫ
        RGBtoHSL(&rgb, &hsl);

        TCS34725_GetRawData_1(&rgb_1);  //�����Σ�ʵ�ʲ���ʱ���ֶ�������ɫ������һ�ε���ɫ
        RGBtoHSL(&rgb_1, &hsl_1);

        TCS34725_GetRawData_2(&rgb_2);  //�����Σ�ʵ�ʲ���ʱ���ֶ�������ɫ������һ�ε���ɫ
        RGBtoHSL(&rgb_2, &hsl_2);

//        TCS34725_GetRawData_3(&rgb_3);  //�����Σ�ʵ�ʲ���ʱ���ֶ�������ɫ������һ�ε���ɫ
//        RGBtoHSL(&rgb_3, &hsl_3);

//        TCS34725_GetRawData_4(&rgb_4);  //�����Σ�ʵ�ʲ���ʱ���ֶ�������ɫ������һ�ε���ɫ
//        RGBtoHSL(&rgb_4, &hsl_4);
    }

    if(hsl.h < 100 && hsl_1.h > 100 && hsl_2.h > 100)
    {
        TIM_SetCompare1(TIM5, 75); //����
    }

    else if(hsl_1.h < 100 && hsl.h > 100 && hsl_2.h > 100)
    {
        guizhong();
//			go_forward();
    }
    else if(hsl_2.h < 100 && hsl.h > 100 && hsl_1.h > 100)
    {
        TIM_SetCompare1(TIM5, 65); //�Ҵ��
    }
//    else if(hsl.h < 100 && hsl_1.h < 100 && hsl_2.h < 100)
//    {
//        hsl_cnt++;

//        if(hsl_cnt > 100)
//        {
//            if(a == 1)
//            {
//                turn_left_flag = 0;
//                TIM_SetCompare1(TIM5, 85);//����
//                delay_ms(1000);
//                TIM_SetCompare1(TIM5, 70);//�����λ

//            }
//            else if(turn_right_flag == 1)
//            {
//                turn_right_flag = 0;
//                TIM_SetCompare1(TIM5, 55);//�Ҵ��
//                delay_ms(1000);
//                TIM_SetCompare1(TIM5, 70);//�����λ
//            }
//            else
//            {
////            go_forward();
//            }
//        }
//    }
    else
    {
        guizhong();
//			go_forward();
    }
}
//����ͷ��ɨ
void scan_left(void)
{
    TIM_SetCompare2(TIM5, 60); //���
}
//����ͷ��ɨ
void scan_right(void)
{
    TIM_SetCompare2(TIM5, 80); //���
}
//����ͷ����
void scan_mid(void)
{
    TIM_SetCompare2(TIM5, 70); //���
}
//��ת����
void turn_left(void)
{
    TIM_SetCompare1(TIM5, 80); //���
}
//��ת����
void turn_right(void)
{
    TIM_SetCompare1(TIM5, 60); //���
}
//���к���
void guizhong(void)
{
    TIM_SetCompare1(TIM5, 70); //���
}
//ǰ������
void go_forward(void)
{
//    if(run_flag == 1)
//    {
        GPIO_SetBits(GPIOC, GPIO_Pin_0); //������ת___AIN2
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);//___AIN1

        GPIO_SetBits(GPIOC, GPIO_Pin_2); //�ҵ����ת__BIN2
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);//__BIN1

        TIM_SetCompare1(TIM3, x1);
        TIM_SetCompare1(TIM4, x1);
//    }
//    else
//    {
//        TIM_SetCompare1(TIM3, 140);
//        TIM_SetCompare1(TIM4, 140);
//    }
}
//���˺���
void go_back(void)
{
    if(run_flag == 1)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_0); //������ת
        GPIO_SetBits(GPIOC, GPIO_Pin_1);

        GPIO_SetBits(GPIOC, GPIO_Pin_2); //�ҵ����ת
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);

        TIM_SetCompare1(TIM3, x1);
        TIM_SetCompare2(TIM3, x1);
    }
    else
    {
        TIM_SetCompare1(TIM3, 140);
        TIM_SetCompare1(TIM4, 140);
    }
}

void stop(void)
{
    z = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6);
    y = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);

    if(z != y)
    {
        run_flag = 0;
    }
    else run_flag = 1;
}
