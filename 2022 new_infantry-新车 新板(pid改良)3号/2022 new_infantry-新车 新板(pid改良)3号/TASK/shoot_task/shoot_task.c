#include "shoot_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
static void Shoot_Feedback_Update(Shoot_Motor_t* shoot_motor_update);//射击数据更新
static void shoot_init(Shoot_Motor_t* shoot_motor_init);//射击初始化
static void shoot_mode_control(Shoot_Motor_t* shoot_mode, remote_mode_e *inputmode); //射击模式控制
static void shoot_normal_control(Shoot_Motor_t* normal_shoot);//手动控制
static void shoot_down_control(Shoot_Motor_t* shoot_down);//停止射击
static void friction_shoot_control(Shoot_Motor_t* shoot_control);//摩擦轮控制
static void trigger_angle_control(Shoot_Motor_t* angle_control);//拨弹轮角度控制
static void trigger_speed_control(Shoot_Motor_t* speed_control);//拨弹轮速度控制
static void shoot_keymouse_control(Shoot_Motor_t* keymouse_shoot);//键鼠射击
static void shoot_ctrl_limit(Shoot_Motor_t* shoot_limits);//枪口热量射速限制
#if (INFANTRY_ID==3)
uint16_t FIRE_RATE_15MS_SET 	=			4250;					//速度上限15
uint16_t FIRE_RATE_18MS_SET 	=		  4640; 				//速度上限18
uint16_t FIRE_RATE_30MS_SET 	=			6900;					//速度上限30
#elif ((INFANTRY_ID==4))
uint16_t FIRE_RATE_15MS_SET   =     4850;					//速度上限15   
uint16_t FIRE_RATE_18MS_SET   =     5370; 				//速度上限18   
uint16_t FIRE_RATE_30MS_SET   =     7800;					//速度上限30   
#elif ((INFANTRY_ID==5))
uint16_t FIRE_RATE_15MS_SET   =     4520;					//速度上限15   
uint16_t FIRE_RATE_18MS_SET   =     4950; 				//速度上限18   
uint16_t FIRE_RATE_30MS_SET   =     7050;					//速度上限30   
//uint16_t FIRE_RATE_15MS_SET = 700;						//速度上限15
//uint16_t FIRE_RATE_18MS_SET = 1100; 						//速度上限18
//uint16_t FIRE_RATE_30MS_SET = 1800;						//速度上限30
#endif
fp32 multiple=1.0f;
#define FIRE_MIN_SET  					2000				//发射要求最低射速限制
//float first_speed=4700,second_speed=5150,thirst_speed=7000;
int32_t TRI_SINGLE_SPEED_SET = 3200;			//单环拨轮转速设置
#define NINE_TRI_ANGLE_SET  		40.0f				//拨弹论单发角度
//遥控输入模式设定
extern remote_mode_e INPUTMOD;
extern u8 RFlag_state;
Shoot_Motor_t shoot_motor;//static
#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t ShootTaskStack;
#endif
int flag_shoot;
void shoot_task(void *pvParameters)
{
    //空闲一段时间
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
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */
static void shoot_init(Shoot_Motor_t* shoot_motor_init)
{
    if (shoot_motor_init == NULL)
    {
        return;
    }

    //单环控制
    static const fp32 Trigger_single_pid[3] = {TRIGGER_SINGLE_PID_KP, TRIGGER_SINGLE_PID_KI, TRIGGER_SINGLE_PID_KD};
    static const fp32 friction_left_pid[3] = {FRICTION_LEFT_PID_KP, FRICTION_LEFT_PID_KI, FRICTION_LEFT_PID_KD};
    static const fp32 friction_right_pid[3] = {FRICTION_RIGHT_PID_KP, FRICTION_RIGHT_PID_KI, FRICTION_RIGHT_PID_KD};
    //双环控制拨弹轮
    fp32 Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
    //获取遥控器指针
    shoot_motor_init->shoot_rc_ctrl = get_remote_control_point();
    //获取电机指针
    shoot_motor_init->trigger_motor_measure = get_Trigger_Motor_Measure_Point();
    shoot_motor_init->trigger_encoder_measure = get_Trigger_Encoder_Measure_Point();

    for(uint8_t i = 0; i < 2; i++)
    {
        shoot_motor_init->friction_motor_measure[i] = get_Friction_Motor_Measure_Point(i);
        shoot_motor_init->friction_encoder_measure[i] = get_Friction_Encoder_Measure_Point(i);
    }

    //获取裁判系统数据
    shoot_motor_init->shoot_data_measure = get_shoot_data_t();
    shoot_motor_init->shoot_heat_measure = get_power_heat_data_t();
    shoot_motor_init->shoot_status_measure = get_game_robot_state_t();
    shoot_motor_init->shoot_hurt_point = get_robot_hurt_t();
		shoot_motor_init->PC_Shoot_Measure_Point = get_PC_Ctrl_Measure_Point();
    //获得监视器数据
    shoot_motor_init->shoot_monitor_point = getErrorListPoint();
    //初始化摩擦轮/拨弹轮射速
    shoot_motor_init->friction_motor_speed_set = 0;
    shoot_motor_init->trigger_motor_speed = 0;
    shoot_motor_init->trigger_motor_set_speed = 0;
    //获取角度
    shoot_motor_init->trigger_motor_set_angle = 0;
    //初始化PID
    PID_Init(&shoot_motor_init->trigger_motor_single_pid, PID_POSITION, Trigger_single_pid, TRIGGER_SINGLE_PID_MAX_OUT, TRIGGER_SINGLE_PID_MAX_IOUT);
    PID_Init(&shoot_motor_init->friction_motor_left_pid, PID_POSITION, friction_left_pid, FRICTION_PID_MAX_OUT, FRICTION_PID_MAX_IOUT);
    PID_Init(&shoot_motor_init->friction_motor_right_pid, PID_POSITION, friction_right_pid, FRICTION_PID_MAX_OUT, FRICTION_PID_MAX_IOUT);
    //初始化双环控制PID
    PID_Init(&shoot_motor_init->trigger_angle_bicyclo_pid, PID_POSITION, Trigger_angle_pid, TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT);
    PID_Init(&shoot_motor_init->trigger_speed_bicyclo_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);
}
/**
  * @brief          射击数据更新
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

    //拨弹轮电机速度滤波一下
//    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

//    //二阶低通滤波
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

    //计算输出轴角度
    shoot_motor_update->trigger_motor_angle = (shoot_motor_update->trigger_motor_measure->all_ecd) * Motor_ECD_TO_ANGLE;//shoot_motor_update->trigger_motor_ecd_count * ecd_range + shoot_motor_update->trigger_motor_measure->ecd
}
/**
  * @brief          射击模式控制
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

    /* 鼠标数据更新
    左键开启/关闭 ———— 开启射击 E 键控制摩擦轮开关
    */
    //比赛优先开启摩擦轮
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
            //读取裁判系统判断当前射速上限
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
            friction_shoot_control(shoot_mode);//开启摩擦轮
        }
        else
        {
            shoot_mode->friction_motor_speed_set = 0;
            friction_shoot_control(shoot_mode);//关闭摩擦轮
        }

        if(!(shoot_mode->shoot_rc_ctrl->mouse.press_l != 0)) //松开鼠标左键停止射击
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
/***************包含有最低速度限制，后期可去********************/
        if(shoot_mode->friction_motor_measure[FRICTION_LEFT]->speed_rpm < -FIRE_MIN_SET && shoot_mode->friction_motor_measure[FRICTION_RIGHT]->speed_rpm > FIRE_MIN_SET)
        {
					     if(shoot_mode->shoot_monitor_point[JudgementWDG].errorExist == 0)
          {
            if(Repeater_flag == 1 && last_Repeater_flag == 0)
            {
							  sum1++;
                RFlag_state = 0;
                shoot_mode->trigger_motor_mode = SINGLE_SHOOT;//单发
                shoot_mode->trigger_motor_set_angle = shoot_mode->trigger_motor_angle + NINE_TRI_ANGLE_SET;
            }
            else  if( (Repeater_flag == 2) && (last_Repeater_flag == 1) )
            {
						
                shoot_mode->trigger_motor_mode = RUNNING_FIRE;//连发
            }
            else if((last_Repeater_flag == 2) && (Repeater_flag == 0)) //停止射击
            {
                shoot_mode->trigger_motor_mode = SHOOT_STOP;
            }
						
//						if(shoot_mode->gimbal_Behaviour_r->gimbal_behaviour == GIMBAL_BIG_BUFF)//能量机关自动射击
//						{
//              if(shoot_mode->gimbal_monitor_point[TX2DataWGD].errorExist == 0)
//							{
//								static int wait_time=0;
//								if(shoot_mode->PC_Shoot_Measure_Point->PcDate.shoot_flag == 1)
//								{
//									if(wait_time == 0)
//									{
//									 shoot_mode->trigger_motor_set_angle = shoot_mode->trigger_motor_angle + NINE_TRI_ANGLE_SET;
//									 wait_time++;          //射击一发后进入短时间等待
//									}
//                  else if(wait_time!=0)  //防止疯狂输出
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

    //遥控射击控制->遥控拨轮向下滚动触发射击
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
        //遥控拨轮向上拨动打开弹舱；拨到底关闭弹舱
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

    //系统掉电模式
    if(*inputmode == RUN_STOP)
    {
        shoot_down_control(shoot_mode);
    }
}
/**
  * @brief          遥控射击
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
  * @brief          键鼠射击
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
fp32 angle_diff;
static void  shoot_keymouse_control(Shoot_Motor_t* keymouse_shoot)
{
    if(keymouse_shoot->trigger_motor_mode == SINGLE_SHOOT)//单发
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
  * @brief          枪口热量限制
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ctrl_limit(Shoot_Motor_t* shoot_limits)
{
    static uint16_t Real_heat = 0;
    static uint16_t Shoot_heat_limit = 0;
    static uint16_t D_value = 0;

    Real_heat = shoot_limits->shoot_heat_measure->shooter_id1_17mm_cooling_heat;//真实热量值
    Shoot_heat_limit = shoot_limits->shoot_status_measure->shooter_id1_17mm_cooling_limit;//枪口热量上限
    D_value = Shoot_heat_limit - Real_heat;//剩余热量

    if(shoot_limits->shoot_monitor_point[JudgementWDG].errorExist == 0)
    {
        u16 Heat_Rec;
        u8 Heat_Flag = 1;

        //设置4发子弹热量的冗余值，防止因拨弹轮转速过快导致超热量
        if(D_value <= 40) /*|| shoot_limits->shoot_hurt_point->hurt_type == 0x3超热量扣血  || shoot_limits->shoot_hurt_point->hurt_type == 0x2超射速扣血*/
        {
            shoot_limits->trigger_motor_set_speed = 0;
            trigger_speed_control(shoot_limits);

            Heat_Flag = 0;
            Heat_Rec++;//缓冲等待
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
  * @brief          摩擦轮单环控制
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
    //线速度控制
    PID_Calc(&shoot_control->friction_motor_left_pid, shoot_control->friction_motor_measure[FRICTION_LEFT]->speed_rpm*multiple, -shoot_control->friction_motor_speed);
    PID_Calc(&shoot_control->friction_motor_right_pid, shoot_control->friction_motor_measure[FRICTION_RIGHT]->speed_rpm*multiple, shoot_control->friction_motor_speed);
    //角速度控制
//	PID_Calc(&shoot_control->friction_motor_left_pid, shoot_control->friction_encoder_measure[FRICTION_LEFT]->filter_rate, -shoot_control->friction_motor_speed);
//	PID_Calc(&shoot_control->friction_motor_right_pid, shoot_control->friction_encoder_measure[FRICTION_RIGHT]->filter_rate, shoot_control->friction_motor_speed);
}
/**
  * @brief          拨弹轮双环角度控制
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
  * @brief          拨弹轮单环速度控制
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
  * @brief          停止射击控制
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
