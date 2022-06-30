#include "shoot_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
static void Shoot_Feedback_Update(Shoot_Motor_t* shoot_motor_update);//������ݸ���
static void shoot_init(Shoot_Motor_t* shoot_motor_init);//�����ʼ��
static void shoot_mode_control(Shoot_Motor_t* shoot_mode, remote_mode_e *inputmode); //���ģʽ����
static void shoot_normal_control(Shoot_Motor_t* normal_shoot);//�ֶ�����
static void shoot_down_control(Shoot_Motor_t* shoot_down);//ֹͣ���
static void friction_shoot_control(Shoot_Motor_t* shoot_control);//Ħ���ֿ���
static void trigger_angle_control(Shoot_Motor_t* angle_control);//�����ֽǶȿ���
static void trigger_speed_control(Shoot_Motor_t* speed_control);//�������ٶȿ���
static void shoot_keymouse_control(Shoot_Motor_t* keymouse_shoot);//�������
static void shoot_ctrl_limit(Shoot_Motor_t* shoot_limits);//ǹ��������������
#if (INFANTRY_ID==3)
uint16_t FIRE_RATE_15MS_SET 	=			4250;					//�ٶ�����15
uint16_t FIRE_RATE_18MS_SET 	=		  4640; 				//�ٶ�����18
uint16_t FIRE_RATE_30MS_SET 	=			6900;					//�ٶ�����30
#elif ((INFANTRY_ID==4))
uint16_t FIRE_RATE_15MS_SET   =     4850;					//�ٶ�����15   
uint16_t FIRE_RATE_18MS_SET   =     5370; 				//�ٶ�����18   
uint16_t FIRE_RATE_30MS_SET   =     7800;					//�ٶ�����30   
#elif ((INFANTRY_ID==5))
uint16_t FIRE_RATE_15MS_SET   =     4520;					//�ٶ�����15   
uint16_t FIRE_RATE_18MS_SET   =     4950; 				//�ٶ�����18   
uint16_t FIRE_RATE_30MS_SET   =     7050;					//�ٶ�����30   
//uint16_t FIRE_RATE_15MS_SET = 700;						//�ٶ�����15
//uint16_t FIRE_RATE_18MS_SET = 1100; 						//�ٶ�����18
//uint16_t FIRE_RATE_30MS_SET = 1800;						//�ٶ�����30
#endif
fp32 multiple=1.0f;
#define FIRE_MIN_SET  					2000				//����Ҫ�������������
//float first_speed=4700,second_speed=5150,thirst_speed=7000;
int32_t TRI_SINGLE_SPEED_SET = 3200;			//��������ת������
#define NINE_TRI_ANGLE_SET  		40.0f				//�����۵����Ƕ�
//ң������ģʽ�趨
extern remote_mode_e INPUTMOD;
extern u8 RFlag_state;
Shoot_Motor_t shoot_motor;//static
#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t ShootTaskStack;
#endif
int flag_shoot;
void shoot_task(void *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    shoot_init(&shoot_motor);

    while(1)
    {
			  Shoot_Feedback_Update(&shoot_motor);
        shoot_mode_control(&shoot_motor, &INPUTMOD);
        
        CAN_CMD_Shoot((int16_t)shoot_motor.trigger_give_current,  shoot_motor.friction_motor_right_pid.out,  shoot_motor.friction_motor_left_pid.out);
        vTaskDelay(1);
        #if INCLUDE_uxTaskGetStackHighWaterMark
        ShootTaskStack = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}
/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @author         RM
  * @param[in]      void
  * @retval         ���ؿ�
  */
static void shoot_init(Shoot_Motor_t* shoot_motor_init)
{
    if (shoot_motor_init == NULL)
    {
        return;
    }

    //��������
    static const fp32 Trigger_single_pid[3] = {TRIGGER_SINGLE_PID_KP, TRIGGER_SINGLE_PID_KI, TRIGGER_SINGLE_PID_KD};
    static const fp32 friction_left_pid[3] = {FRICTION_LEFT_PID_KP, FRICTION_LEFT_PID_KI, FRICTION_LEFT_PID_KD};
    static const fp32 friction_right_pid[3] = {FRICTION_RIGHT_PID_KP, FRICTION_RIGHT_PID_KI, FRICTION_RIGHT_PID_KD};
    //˫�����Ʋ�����
    fp32 Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
    //��ȡң����ָ��
    shoot_motor_init->shoot_rc_ctrl = get_remote_control_point();
    //��ȡ���ָ��
    shoot_motor_init->trigger_motor_measure = get_Trigger_Motor_Measure_Point();
    shoot_motor_init->trigger_encoder_measure = get_Trigger_Encoder_Measure_Point();

    for(uint8_t i = 0; i < 2; i++)
    {
        shoot_motor_init->friction_motor_measure[i] = get_Friction_Motor_Measure_Point(i);
        shoot_motor_init->friction_encoder_measure[i] = get_Friction_Encoder_Measure_Point(i);
    }

    //��ȡ����ϵͳ����
    shoot_motor_init->shoot_data_measure = get_shoot_data_t();
    shoot_motor_init->shoot_heat_measure = get_power_heat_data_t();
    shoot_motor_init->shoot_status_measure = get_game_robot_state_t();
    shoot_motor_init->shoot_hurt_point = get_robot_hurt_t();
		shoot_motor_init->PC_Shoot_Measure_Point = get_PC_Ctrl_Measure_Point();
    //��ü���������
    shoot_motor_init->shoot_monitor_point = getErrorListPoint();
    //��ʼ��Ħ����/����������
    shoot_motor_init->friction_motor_speed_set = 0;
    shoot_motor_init->trigger_motor_speed = 0;
    shoot_motor_init->trigger_motor_set_speed = 0;
    //��ȡ�Ƕ�
    shoot_motor_init->trigger_motor_set_angle = 0;
    //��ʼ��PID
    PID_Init(&shoot_motor_init->trigger_motor_single_pid, PID_POSITION, Trigger_single_pid, TRIGGER_SINGLE_PID_MAX_OUT, TRIGGER_SINGLE_PID_MAX_IOUT);
    PID_Init(&shoot_motor_init->friction_motor_left_pid, PID_POSITION, friction_left_pid, FRICTION_PID_MAX_OUT, FRICTION_PID_MAX_IOUT);
    PID_Init(&shoot_motor_init->friction_motor_right_pid, PID_POSITION, friction_right_pid, FRICTION_PID_MAX_OUT, FRICTION_PID_MAX_IOUT);
    //��ʼ��˫������PID
    PID_Init(&shoot_motor_init->trigger_angle_bicyclo_pid, PID_POSITION, Trigger_angle_pid, TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT);
    PID_Init(&shoot_motor_init->trigger_speed_bicyclo_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);
}
/**
  * @brief          ������ݸ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(Shoot_Motor_t* shoot_motor_update)
{
    if (shoot_motor_update == NULL)
    {
        return;
    }

//    static fp32 speed_fliter_1 = 0.0f;
//    static fp32 speed_fliter_2 = 0.0f;
//    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
//    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

//    //���׵�ͨ�˲�
//    speed_fliter_1 = speed_fliter_2;
//    speed_fliter_2 = speed_fliter_3;
//    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_motor_update->trigger_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
//
//    shoot_motor_update->trigger_motor_speed = speed_fliter_3;

    if (shoot_motor_update->trigger_motor_measure->ecd - shoot_motor_update->trigger_motor_measure->last_ecd > Half_ecd_range)
    {
        shoot_motor_update->trigger_motor_ecd_count--;
    }
    else if (shoot_motor_update->trigger_motor_measure->ecd - shoot_motor_update->trigger_motor_measure->last_ecd < -Half_ecd_range)
    {
        shoot_motor_update->trigger_motor_ecd_count++;
    }

    //���������Ƕ�
    shoot_motor_update->trigger_motor_angle = (shoot_motor_update->trigger_motor_measure->all_ecd) * Motor_ECD_TO_ANGLE;//shoot_motor_update->trigger_motor_ecd_count * ecd_range + shoot_motor_update->trigger_motor_measure->ecd
}
/**
  * @brief          ���ģʽ����
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
u8 last_Repeater_flag = 0, Repeater_flag = 0, loading_flag = 2, last_loading_flag = 0,sum1=0,sum2=0;
fp32 last_bullet_speed;
static void shoot_mode_control(Shoot_Motor_t* shoot_mode, remote_mode_e *inputmode)
{
    static int MOU_LEFT_state = 0;
    static u8 EFlag_state = 0;

    if (shoot_mode == NULL)
    {
        return;
    }

    /* ������ݸ���
    �������/�ر� �������� ������� E ������Ħ���ֿ���
    */
    //�������ȿ���Ħ����
    if(*inputmode == KEYMOUSE_INPUT)
    {
        if(!((shoot_mode->shoot_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E) != 0))
        {
            E_Flag = 1;
        }

        if(shoot_mode->shoot_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E && E_Flag == 1)
        {
            E_Flag = 0;
            EFlag_state++;
            EFlag_state %= 2;
        }

        if(EFlag_state)
        {
            //��ȡ����ϵͳ�жϵ�ǰ��������
            if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit == 15)//15m/s
            {
                if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit - shoot_mode->shoot_data_measure->bullet_speed <= 0.3f
                        && last_bullet_speed != shoot_mode->shoot_data_measure->bullet_speed)
                {
									if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit - shoot_mode->shoot_data_measure->bullet_speed>=0)
									{
									FIRE_RATE_15MS_SET = FIRE_RATE_15MS_SET - 10.0f;
									}//100*(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit - shoot_mode->shoot_data_measure->bullet_speed)/3
                   else
									 {
									 FIRE_RATE_15MS_SET = FIRE_RATE_15MS_SET-(shoot_mode->shoot_data_measure->bullet_speed-shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit)*200;
									 }
								}
                else
                {
                    shoot_mode->friction_motor_speed_set = FIRE_RATE_15MS_SET;
                }

                //RAMP_float(FIRE_RATE_15MS_SET,shoot_mode->friction_motor_speed_set,FRI_RAMP_RATE);
            }
            else if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit == 18)//18m/s
            {
                if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit - shoot_mode->shoot_data_measure->bullet_speed <= 0.3f
                        && last_bullet_speed != shoot_mode->shoot_data_measure->bullet_speed)
                {
									if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit - shoot_mode->shoot_data_measure->bullet_speed>=0)
                    {									
											FIRE_RATE_18MS_SET = FIRE_RATE_18MS_SET - 10.0f;
										}
										else 
										{
										  FIRE_RATE_18MS_SET = FIRE_RATE_18MS_SET-(shoot_mode->shoot_data_measure->bullet_speed-shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit)*200;
										}
                }
                else
                {
                    shoot_mode->friction_motor_speed_set = FIRE_RATE_18MS_SET;
                }

                //RAMP_float(FIRE_RATE_18MS_SET,shoot_mode->friction_motor_speed_set,FRI_RAMP_RATE);
            }
            else if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit == 30)//30m/s
            {
                if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit - shoot_mode->shoot_data_measure->bullet_speed <= 0.3f
                        && last_bullet_speed != shoot_mode->shoot_data_measure->bullet_speed)
                {
									if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit - shoot_mode->shoot_data_measure->bullet_speed>=0)
									{
                    FIRE_RATE_30MS_SET = FIRE_RATE_30MS_SET - 10.0f;
									}
									else 
									{
									  FIRE_RATE_30MS_SET = FIRE_RATE_30MS_SET -(shoot_mode->shoot_data_measure->bullet_speed-shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit)*200;
									}
                }
                else
                {
                    shoot_mode->friction_motor_speed_set = FIRE_RATE_30MS_SET;
                }

                //RAMP_float(FIRE_RATE_30MS_SET,shoot_mode->friction_motor_speed_set,FRI_RAMP_RATE);
            }
            else
            {
                shoot_mode->friction_motor_speed_set = FIRE_RATE_15MS_SET;
                //RAMP_float(FIRE_RATE_15MS_SET,shoot_mode->friction_motor_speed_set,FRI_RAMP_RATE);
            }

            last_bullet_speed = shoot_mode->shoot_data_measure->bullet_speed;
            friction_shoot_control(shoot_mode);//����Ħ����
        }
        else
        {
            shoot_mode->friction_motor_speed_set = 0;
            friction_shoot_control(shoot_mode);//�ر�Ħ����
        }

        if(!(shoot_mode->shoot_rc_ctrl->mouse.press_l != 0)) //�ɿ�������ֹͣ���
        {
            MOU_LEFT_F = 1;
            MOU_LEFT_state = 0;
            Repeater_flag = 0;
        }

        if(shoot_mode->shoot_rc_ctrl->mouse.press_l == 1)
        {
            MOU_RIGH_F = 0;
            MOU_LEFT_state ++;

            if(MOU_LEFT_state <= 150)
            {
                Repeater_flag = 1;
            }
            else if (MOU_LEFT_state > 150)
            {
                Repeater_flag = 2;
            }
        }
/***************����������ٶ����ƣ����ڿ�ȥ********************/
        if(shoot_mode->friction_motor_measure[FRICTION_LEFT]->speed_rpm < -FIRE_MIN_SET && shoot_mode->friction_motor_measure[FRICTION_RIGHT]->speed_rpm > FIRE_MIN_SET)
        {
					     if(shoot_mode->shoot_monitor_point[JudgementWDG].errorExist == 0)
          {
            if(Repeater_flag == 1 && last_Repeater_flag == 0)
            {
							  sum1++;
                RFlag_state = 0;
                shoot_mode->trigger_motor_mode = SINGLE_SHOOT;//����
                shoot_mode->trigger_motor_set_angle = shoot_mode->trigger_motor_angle + NINE_TRI_ANGLE_SET;
            }
            else  if( (Repeater_flag == 2) && (last_Repeater_flag == 1) )
            {
						
                shoot_mode->trigger_motor_mode = RUNNING_FIRE;//����
            }
            else if((last_Repeater_flag == 2) && (Repeater_flag == 0)) //ֹͣ���
            {
                shoot_mode->trigger_motor_mode = SHOOT_STOP;
            }
						
//						if(shoot_mode->gimbal_Behaviour_r->gimbal_behaviour == GIMBAL_BIG_BUFF)//���������Զ����
//						{
//              if(shoot_mode->gimbal_monitor_point[TX2DataWGD].errorExist == 0)
//							{
//								static int wait_time=0;
//								if(shoot_mode->PC_Shoot_Measure_Point->PcDate.shoot_flag == 1)
//								{
//									if(wait_time == 0)
//									{
//									 shoot_mode->trigger_motor_set_angle = shoot_mode->trigger_motor_angle + NINE_TRI_ANGLE_SET;
//									 wait_time++;          //���һ��������ʱ��ȴ�
//									}
//                  else if(wait_time!=0)  //��ֹ������
//									{
//										wait_time++;
//										if(wait_time == 200)
//										{
//											wait_time=0;
//										}
//									}
//								}
//							}								
//						}
					}
					else if(shoot_mode->shoot_monitor_point[JudgementWDG].errorExist == 1)
					{
							sum2++;
					 shoot_mode->trigger_motor_mode = SHOOT_STOP;
					}
        }
        else
        {
            shoot_mode->trigger_motor_mode = SHOOT_STOP;
        }

        last_Repeater_flag = Repeater_flag;
        shoot_ctrl_limit(shoot_mode);
    }

    //ң���������->ң�ز������¹����������
    if(*inputmode == REMOTE_INPUT)
    {
			
        last_loading_flag = loading_flag;

        if(shoot_mode->shoot_rc_ctrl->rc.ch[4] > 0)
        {
            TIM_SetCompare1(TIM5, PWM);
            /**************************/
//
					if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit == 15)
			{
				shoot_mode->friction_motor_speed_set = FIRE_RATE_15MS_SET;
			}
				else if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit == 18)
			{
				shoot_mode->friction_motor_speed_set = FIRE_RATE_18MS_SET;
			}
				else if(shoot_mode->shoot_status_measure->shooter_id1_17mm_speed_limit == 30)
			{
				shoot_mode->friction_motor_speed_set = FIRE_RATE_30MS_SET;
			}
			else
				shoot_mode->friction_motor_speed_set = FIRE_RATE_15MS_SET;
			/*************************/					
            friction_shoot_control(shoot_mode);

            if(shoot_mode->shoot_rc_ctrl->rc.ch[4] > 300)
            {
                shoot_mode->trigger_motor_set_speed = 3200;
                shoot_ctrl_limit(shoot_mode);
            }
            else if(shoot_mode->shoot_rc_ctrl->rc.ch[4] < 300)
            {
                shoot_mode->trigger_motor_set_speed = 0;
                shoot_ctrl_limit(shoot_mode);
            }
        }
        //ң�ز������ϲ����򿪵��գ������׹رյ���
        else if(shoot_mode->shoot_rc_ctrl->rc.ch[4] < 0 && shoot_mode->shoot_rc_ctrl->rc.ch[4] > -660)
        {
            loading_flag = 1;
        }
        else if(shoot_mode->shoot_rc_ctrl->rc.ch[4] == -660)
        {
            loading_flag = 2;
        }
        else
        {
            loading_flag = 0;
            shoot_mode->friction_motor_speed_set = 0;
            friction_shoot_control(shoot_mode);
            shoot_mode->trigger_motor_set_speed = 0;
            shoot_ctrl_limit(shoot_mode);
        }

        if(loading_flag == 1 && last_loading_flag == 0)
        {
            TIM_SetCompare1(TIM5, 19490);
            shoot_mode->friction_motor_speed_set = 0;
            friction_shoot_control(shoot_mode);
            shoot_mode->trigger_motor_set_speed = 0;
            shoot_ctrl_limit(shoot_mode);
        }
        else if(loading_flag == 2 && last_loading_flag == 1)
        {
            TIM_SetCompare1(TIM5, PWM);
            shoot_mode->friction_motor_speed_set = 0;
            friction_shoot_control(shoot_mode);
            shoot_mode->trigger_motor_set_speed = 0;
            shoot_ctrl_limit(shoot_mode);
        }
    }

    //ϵͳ����ģʽ
    if(*inputmode == RUN_STOP)
    {
        shoot_down_control(shoot_mode);
    }
}
/**
  * @brief          ң�����
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void  shoot_normal_control(Shoot_Motor_t* normal_shoot)
{
    if (normal_shoot == NULL)
    {
        return;
    }

    trigger_speed_control(normal_shoot);
}
/**
  * @brief          �������
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
fp32 angle_diff;
static void  shoot_keymouse_control(Shoot_Motor_t* keymouse_shoot)
{
    if(keymouse_shoot->trigger_motor_mode == SINGLE_SHOOT)//����
    {
        trigger_angle_control(keymouse_shoot);
        angle_diff = keymouse_shoot->trigger_motor_angle - keymouse_shoot->trigger_motor_set_angle;

        if(fabs(angle_diff) <= 0.1f)
        {
            keymouse_shoot->trigger_motor_mode = SHOOT_STOP;
        }
    }
    else if(keymouse_shoot->trigger_motor_mode == RUNNING_FIRE)
    {
        keymouse_shoot->trigger_motor_set_speed = TRI_SINGLE_SPEED_SET;
        trigger_speed_control(keymouse_shoot);
    }
    else
    {
        keymouse_shoot->trigger_motor_set_speed = 0;
        trigger_speed_control(keymouse_shoot);
    }
}
/**
  * @brief          ǹ����������
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ctrl_limit(Shoot_Motor_t* shoot_limits)
{
    static uint16_t Real_heat = 0;
    static uint16_t Shoot_heat_limit = 0;
    static uint16_t D_value = 0;

    Real_heat = shoot_limits->shoot_heat_measure->shooter_id1_17mm_cooling_heat;//��ʵ����ֵ
    Shoot_heat_limit = shoot_limits->shoot_status_measure->shooter_id1_17mm_cooling_limit;//ǹ����������
    D_value = Shoot_heat_limit - Real_heat;//ʣ������

    if(shoot_limits->shoot_monitor_point[JudgementWDG].errorExist == 0)
    {
        u16 Heat_Rec;
        u8 Heat_Flag = 1;

        //����4���ӵ�����������ֵ����ֹ�򲦵���ת�ٹ��쵼�³�����
        if(D_value <= 40) /*|| shoot_limits->shoot_hurt_point->hurt_type == 0x3��������Ѫ  || shoot_limits->shoot_hurt_point->hurt_type == 0x2�����ٿ�Ѫ*/
        {
            shoot_limits->trigger_motor_set_speed = 0;
            trigger_speed_control(shoot_limits);

            Heat_Flag = 0;
            Heat_Rec++;//����ȴ�
        }
        else if(Heat_Flag == 1 || Heat_Rec == 300)
        {
            if( INPUTMOD == KEYMOUSE_INPUT)
            {
                shoot_keymouse_control(shoot_limits);
            }

            if( INPUTMOD == REMOTE_INPUT)
            {
                shoot_normal_control(shoot_limits);
            }

            Heat_Rec = 0;
        }
    }
    else if(shoot_limits->shoot_monitor_point[JudgementWDG].errorExist == 1)
    {
//			shoot_limits->trigger_motor_mode = SHOOT_STOP;
        if( INPUTMOD == KEYMOUSE_INPUT)
        {
            shoot_keymouse_control(shoot_limits);
        }

        if( INPUTMOD == REMOTE_INPUT)
        {
            shoot_normal_control(shoot_limits);
        }
    }
}
/**
  * @brief          Ħ���ֵ�������
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void friction_shoot_control(Shoot_Motor_t* shoot_control)
{
    if (shoot_control == NULL)
    {
        return;
    }

    shoot_control->friction_motor_speed = shoot_control->friction_motor_speed_set;
    //���ٶȿ���
    PID_Calc(&shoot_control->friction_motor_left_pid, shoot_control->friction_motor_measure[FRICTION_LEFT]->speed_rpm*multiple, -shoot_control->friction_motor_speed);
    PID_Calc(&shoot_control->friction_motor_right_pid, shoot_control->friction_motor_measure[FRICTION_RIGHT]->speed_rpm*multiple, shoot_control->friction_motor_speed);
    //���ٶȿ���
//	PID_Calc(&shoot_control->friction_motor_left_pid, shoot_control->friction_encoder_measure[FRICTION_LEFT]->filter_rate, -shoot_control->friction_motor_speed);
//	PID_Calc(&shoot_control->friction_motor_right_pid, shoot_control->friction_encoder_measure[FRICTION_RIGHT]->filter_rate, shoot_control->friction_motor_speed);
}
/**
  * @brief          ������˫���Ƕȿ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void trigger_angle_control(Shoot_Motor_t* angle_control)
{
    PID_Calc(&angle_control->trigger_angle_bicyclo_pid,	angle_control->trigger_motor_angle, angle_control->trigger_motor_set_angle);
    angle_control->trigger_give_current =  PID_Calc(&angle_control->trigger_speed_bicyclo_pid, angle_control->trigger_motor_measure->speed_rpm,
                                           angle_control->trigger_angle_bicyclo_pid.out);
}
/**
  * @brief          �����ֵ����ٶȿ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void trigger_speed_control(Shoot_Motor_t* speed_control)
{
    speed_control->trigger_give_current = PID_Calc(&speed_control->trigger_motor_single_pid, speed_control->trigger_motor_measure->speed_rpm,
                                          speed_control->trigger_motor_set_speed);
}
/**
  * @brief          ֹͣ�������
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_down_control(Shoot_Motor_t* shoot_down)
{
    shoot_down->friction_motor_speed = 0.0f;
    shoot_down->trigger_motor_set_speed = 0.0f;

    PID_Calc(&shoot_down->friction_motor_left_pid, shoot_down->friction_motor_measure[FRICTION_LEFT]->speed_rpm, -shoot_down->friction_motor_speed);
    PID_Calc(&shoot_down->friction_motor_right_pid, shoot_down->friction_motor_measure[FRICTION_RIGHT]->speed_rpm, shoot_down->friction_motor_speed);
    shoot_down->trigger_give_current = PID_Calc(&shoot_down->trigger_motor_single_pid, shoot_down->trigger_motor_measure->speed_rpm, shoot_down->trigger_motor_set_speed);
}
