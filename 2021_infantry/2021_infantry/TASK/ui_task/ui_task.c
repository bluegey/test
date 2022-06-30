#include "ui_task.h"
#include "RM_Cilent_UI.h"
#include "string.h"

Graph_Data G5, G8, G10, G11, G12, G21, G22, G23, G24, G25, G26, G40, G41, G42, G43, G44;
Float_Data G6;
fp32 yaw_angle, pitch_angle, x1_ui, y1_ui, x0_ui, y0_ui, sin_ui, cos_ui, speed, superpower_x, super_ratio;
ui_t ui;
uint32_t UiTaskStack;
static void ui_data_update(ui_t *ui_init);
extern u32 Robot_ID, Cilent_ID;
extern u8 RFlag_state;
extern chassis_move_t chassis_move;
extern Gimbal_Control_t gimbal_control;//static
static void ID_change(ui_t *ui_init);
String_Data CH_FRICTION, CH_GIMBAL, CH_BUFF, CH_CHASSIC, CH_BULLET, SUPER;
char		 	Friction[9] = "FRICTION:", Spin[5] = "SPIN:", Auto_aim[9] = "AUTO_AIM:", Shoot_buff[5] = "BUFF:", Magazine[9] = "MAGAZINE:";
char 	 Super_cap[5] = {0};
uint8_t VFlag_state;
void ui_task(void  *pvParameters)
{
    //��������
    memset(&G5, 0, sizeof(G5));
    //����λ��
    memset(&G8, 0, sizeof(G8));
    //����λ��
    memset(&G10, 0, sizeof(G10));
    memset(&G11, 0, sizeof(G11));
    memset(&G12, 0, sizeof(G12));
    //׼��
    memset(&G21, 0, sizeof(G21));
    memset(&G22, 0, sizeof(G22));
    memset(&G23, 0, sizeof(G23));
    memset(&G24, 0, sizeof(G24));
    memset(&G25, 0, sizeof(G25));
		memset(&G26, 0, sizeof(G26));
    //������̬
    memset(&CH_CHASSIC, 0, sizeof(CH_CHASSIC));
    memset(&CH_GIMBAL, 0, sizeof(CH_GIMBAL));
		memset(&CH_BUFF, 0, sizeof(CH_BUFF));
    memset(&CH_FRICTION, 0, sizeof(CH_FRICTION));
		memset(&CH_BULLET, 0, sizeof(CH_BULLET));
    memset(&SUPER, 0, sizeof(SUPER));


		Line_Draw(&G21, "075", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 900, 405, 1020, 405); //׼��---->2m,3m
    Line_Draw(&G22, "076", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 910, 390, 1010, 390); //׼��----->1m
		Line_Draw(&G23, "077", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 920, 375, 1000, 375); //׼��---->4m
		Line_Draw(&G24, "078", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 930, 345, 990, 345);//׼��----->5m
    Line_Draw(&G25, "079", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 940, 300, 980, 300); //׼��---->6m
		Line_Draw(&G26, "080", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 960, 405, 960, 300); //׼������

    Line_Draw(&G10, "081", UI_Graph_ADD, 0, UI_Color_White, 3, 620, 0, 770, 300); //���̿ɹ�λ�����־λ
    Line_Draw(&G11, "082", UI_Graph_ADD, 0, UI_Color_White, 3, 1220, 0, 1060, 300); //���̿ɹ�λ���ұ�־λ
    Line_Draw(&G12, "083", UI_Graph_ADD, 0, UI_Color_White, 3, 800, 177, 1120, 177); //�������ػ����־λ

		Line_Draw(&G40, "084", UI_Graph_ADD, 0, UI_Color_White, 30, 430, 800, 460, 800);//BUFF
		Line_Draw(&G41, "085", UI_Graph_ADD, 0, UI_Color_White, 30, 430, 750, 460, 750);//����
		Line_Draw(&G42, "086", UI_Graph_ADD, 0, UI_Color_White, 30, 430, 700, 460, 700);//Ħ����
		Line_Draw(&G43, "087", UI_Graph_ADD, 0, UI_Color_White, 30, 430, 650, 460, 650);//����
		Line_Draw(&G44, "088", UI_Graph_ADD, 0, UI_Color_White, 30, 430, 600, 460, 600);//����

    Line_Draw(&G8, "101", UI_Graph_ADD, 1, UI_Color_Green, 2, 960, 620, 960, 660); //����λ��
    Char_Draw(&CH_FRICTION, "102", UI_Graph_ADD, 1, UI_Color_Main, 20, 9, 1, 250, 710, &Friction[0]); //Ħ����״̬
    Char_Draw(&CH_CHASSIC, "103", UI_Graph_ADD, 1, UI_Color_Main, 20, 5, 1, 330, 760, &Spin[0]); //����״̬
    Char_Draw(&CH_GIMBAL, "104", UI_Graph_ADD, 1, UI_Color_Main, 20, 9, 1, 250, 660, &Auto_aim[0]); //����
    Char_Draw(&CH_BULLET, "105", UI_Graph_ADD, 1, UI_Color_Main, 20, 9, 1, 250, 610, &Magazine[0]); //����״̬
    Char_Draw(&SUPER, "106", UI_Graph_ADD, 1, UI_Color_Orange, 40, 5, 1, 1500, 830, &Super_cap[0]); //�������ݡ����ٷֱ�
    Line_Draw(&G5, "107", UI_Graph_ADD, 1, UI_Color_Orange, 15, 700, 100, (Super_power.volt - 17000) / 10 + 700, 100); //�������ݡ���������
		Char_Draw(&CH_BUFF, "109", UI_Graph_ADD, 1, UI_Color_Main, 20, 5, 1, 330, 810, &Shoot_buff[0]); //���
		
    UI_ReFresh(5, G5, G8, G10, G11, G12);
    UI_ReFresh(7, G21, G22, G23, G24, G25);
		UI_ReFresh(1, G26);
		UI_ReFresh(5, G40, G41, G42, G43, G44);
		
    Char_ReFresh(CH_GIMBAL);
    Char_ReFresh(CH_FRICTION);
    Char_ReFresh(CH_CHASSIC);
		Char_ReFresh(CH_BUFF);
    Char_ReFresh(CH_BULLET);
    Char_ReFresh(SUPER);
		
    while(1)
    {
        ui_data_update(&ui);

        //V�����´�UI
        if((KEY_PRESSED_OFFSET_V & ui.ui_remote_ctrl->key.v) != 0 )
        {
            V_Flag = 1;
        }

        if(KEY_PRESSED_OFFSET_V & ui.ui_remote_ctrl->key.v && V_Flag == 1)
        {
            V_Flag = 0;
            VFlag_state ++;
            VFlag_state %= 2;
        }

        if(VFlag_state)
        {
            ID_change(&ui);
            /*-------------------------------------------------------UI����---------------------------------------------------------------*/
            UI_ReFresh(5, G5, G8, G10, G11, G12);
						UI_ReFresh(7, G21, G22, G23, G24, G25);
						UI_ReFresh(1, G26);
						UI_ReFresh(5, G40, G41, G42, G43, G44);

            Char_ReFresh(CH_GIMBAL);
            Char_ReFresh(CH_FRICTION);
            Char_ReFresh(CH_CHASSIC);
						Char_ReFresh(CH_BUFF);
            Char_ReFresh(CH_BULLET);
            Char_ReFresh(SUPER);

            /*-------------------------------------------------------����λ��---------------------------------------------------------------*/
            x0_ui = 960.0f + arm_sin_f32(yaw_angle) * 80.0f;
            y0_ui = arm_cos_f32(yaw_angle) * 80.0f + 540.0f;
            x1_ui = 960.0f + arm_sin_f32(yaw_angle) * 120.0f;
            y1_ui = arm_cos_f32(yaw_angle) * 120.0f + 540.0f;
            Line_Draw(&G8, "101", UI_Graph_Change, 1, UI_Color_Green, 3, x0_ui, y0_ui, x1_ui, y1_ui); //����λ��
            /*-------------------------------------------------------��������---------------------------------------------------------------*/
            superpower_x = Super_power.volt;

            if(superpower_x >= 22000.0f)
            {
                superpower_x = 22000.0f;
            }
            else if(superpower_x <= 17000.0f)
            {
                superpower_x = 17000.0f;
            }

            super_ratio = (superpower_x - 17000.0f) / 5000.0f * 100.0f;
            sprintf(Super_cap, "%0.3f", super_ratio);
            Char_Draw(&SUPER, "106", UI_Graph_Change, 1, UI_Color_Orange, 40, 5, 5, 1500, 830, &Super_cap[0]);
            Line_Draw(&G5, "107", UI_Graph_Change, 1, UI_Color_Orange, 15, 700, 100, (superpower_x - 17000) / 10 + 700, 100);

            /*-------------------------------------------------------����ģʽ---------------------------------------------------------------*/
            if(chassis_move.chassis_mode == CHASSIS_FOLLOW_GIMBAL)
            {
                //���̸���ģʽ
                memset(&CH_CHASSIC, 0, sizeof(CH_CHASSIC));
                Line_Draw(&G41, "085", UI_Graph_Change, 0, UI_Color_Purplish_red, 30, 430, 750, 460, 750);
            }
            if(chassis_move.chassis_mode == CHASSIS_DODGE_MODE)
            {
                //��������ģʽ
                memset(&CH_CHASSIC, 0, sizeof(CH_CHASSIC));
                Line_Draw(&G41, "085", UI_Graph_Change, 0, UI_Color_Green, 30, 430, 750, 460, 750);
            }

            /*-------------------------------------------------------��̨ģʽ---------------------------------------------------------------*/
            if(gimbal_control.gimbal_behaviour == GIMBAL_SHOOT_BUFF)
            {
                //��̨������������ģʽ
                memset(&CH_BUFF, 0, sizeof(CH_BUFF));
                Line_Draw(&G40, "084", UI_Graph_Change, 0, UI_Color_Green, 30, 430, 800, 460, 800);
            }
            else if(gimbal_control.gimbal_behaviour == GIMBAL_TRACK_ARMOR)
            {
                //��̨׷��װ�װ�ģʽ
                memset(&CH_GIMBAL, 0, sizeof(CH_GIMBAL));
                Line_Draw(&G43, "087", UI_Graph_Change, 0, UI_Color_Green, 30, 430, 650, 460, 650);
            }
            else if(gimbal_control.gimbal_behaviour == GIMBAL_MANUAL_MODE)
            {
                //��̨�ֿ�ģʽ
								memset(&CH_BUFF, 0, sizeof(CH_BUFF));
                memset(&CH_GIMBAL, 0, sizeof(CH_GIMBAL));
								Line_Draw(&G40, "084", UI_Graph_Change, 0, UI_Color_Purplish_red, 30, 430, 800, 460, 800);
								Line_Draw(&G43, "087", UI_Graph_Change, 0, UI_Color_Purplish_red, 30, 430, 650, 460, 650);
            }

            /*-------------------------------------------------------Ħ����---------------------------------------------------------------*/
            if(speed > 2000)
            {
                //��Ħ����
                memset(&CH_FRICTION, 0, sizeof(CH_FRICTION));
                Line_Draw(&G42, "086", UI_Graph_Change, 0, UI_Color_Green, 30, 430, 700, 460, 700);
            }
            else
            {
                //�ر�Ħ����
                memset(&CH_FRICTION, 0, sizeof(CH_FRICTION));
								Line_Draw(&G42, "086", UI_Graph_Change, 0, UI_Color_Purplish_red, 30, 430, 700, 460, 700);
            }

            /*-------------------------------------------------------����---------------------------------------------------------------*/
            if(RFlag_state == 0)
            {
                //���չ�
                memset(&CH_BULLET, 0, sizeof(CH_BULLET));
                Line_Draw(&G44, "088", UI_Graph_Change, 0, UI_Color_Purplish_red, 30, 430, 600, 460, 600);
            }
            else if(RFlag_state == 1)
            {
                //���տ�
                memset(&CH_BULLET, 0, sizeof(CH_BULLET));
                Line_Draw(&G44, "088", UI_Graph_Change, 0, UI_Color_Green, 30, 430, 600, 460, 600);
            }
        }

        vTaskDelay(100);
        UiTaskStack = uxTaskGetStackHighWaterMark(NULL);
    }
}

static void ui_data_update(ui_t *ui_init)
{
 
    ui_init->ui_remote_ctrl = get_remote_control_point();   //��ȡң����ָ��
    ui_init->shoot_hurt_point = get_robot_hurt_t(); //��ȡ����ϵͳָ��
    ui_init->ui_yaw_motor = get_yaw_motor_point();//��ȡyaw������
    yaw_angle = ui_init->ui_yaw_motor->relative_angle * UI_ANGLE_TO_RAD;

    for(uint8_t i = 0; i < 2; i++)
    {
        ui_init->friction_motor_measure[i] = get_Friction_Motor_Measure_Point(i);
    }
    speed = ui_init->friction_motor_measure[0]->speed_rpm;
    
    ui_init->ui_pitch_motor = get_pitch_motor_point();//��ȡpitch������
    pitch_angle = ui_init->ui_pitch_motor->relative_angle;
}

static void ID_change(ui_t *ui_init)
{
    ui_init->chassis_status_measure = get_game_robot_state_t();

    if(ui_init->chassis_status_measure->robot_id == 3)
    {
        Robot_ID = 3;
        Cilent_ID = 0x0103;
    }

    if(ui_init->chassis_status_measure->robot_id == 4)
    {
        Robot_ID = 4;
        Cilent_ID = 0x0104;
    }

    if(ui_init->chassis_status_measure->robot_id == 5)
    {
        Robot_ID = 5;
        Cilent_ID = 0x0105;
    }

    if(ui_init->chassis_status_measure->robot_id == 103)
    {
        Robot_ID = 103;
        Cilent_ID = 0x0167;
    }

    if(ui_init->chassis_status_measure->robot_id == 104)
    {
        Robot_ID = 104;
        Cilent_ID = 0x0168;
    }

    if(ui_init->chassis_status_measure->robot_id == 105)
    {
        Robot_ID = 105;
        Cilent_ID = 0x0169;
    }
}







