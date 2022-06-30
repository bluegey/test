/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       预编译：编码器中值，PID。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020     		RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */
#include "gimbal_task.h"

static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);//云台初始化
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);//云台数据更新
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set, remote_mode_e *inputmode); //云台行为设置
static void GIMBAL_Behaviour_update(Gimbal_Control_t *gimbal_Behaviour);//云台行为更新
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);//得到相对角度
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);//云台状态切换保存，用于状态切换过渡
static void gimbal_zero_force(Gimbal_Control_t *gimbal_motor);//掉电模式
static void gimbal_behaviour_init(Gimbal_Control_t *gimbal_motor);//云台初始化模式
static void gimbal_manual_control(Gimbal_Control_t *gimbal_motor);//手动控制模式
static void gimbal_track_armor(Gimbal_Control_t *gimbal_motor);//自瞄模式
static void gimbal_shoot_buff(Gimbal_Control_t *gimbal_motor);//射符模式
static void gimbal_loading_bullet(Gimbal_Control_t *gimbal_motor);//装弹模式
static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);//云台PID初始化
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, extKalman_t Gim); //PID解算
static void gimbal_pid_reset(Gimbal_PID_t *pid, float p, float i, float d);//PID重设
static void gimbal_relative_angle_control(Gimbal_Motor_t *gimbal_motor);//云台相对角度控制
static void gimbal_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);//云台绝对角度控制
float re_yaw_absolute_angle;
float re_pitch_relative_angle;
Forecast  forecast_auxiliary_aim;
Forecast  forecast_auxiliary_buff;
#if gimbal_debug_start
    static void gimbal_debug_mode(Gimbal_Control_t *gimbal_motor);
#endif
//云台控制所有相关数据
Gimbal_Control_t gimbal_control;//static
//遥控输入模式设定
extern unsigned char  Forecast_or_not;
extern remote_mode_e INPUTMOD;
uint8_t Z_Flag = 0,X_Flag=0,C_Flag=0;
u8 RFlag_state;
//发送的can 指令
int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0;//static

//是否使用陀螺仪控制
int8_t YAW_ABS_CTRL 	=	0;
int8_t PITCH_ABS_CTRL 	=	0;
extKalman_t  kalman_yaw;
extKalman_t  kalman_pitch;
extKalman_t  kalman_speed;
int8_t work_turn, type_turn;
extern uint8_t FFlag_state;
#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t GimbalTaskStack;
#endif
#if gimbal_debug_start //debug模式
    fp32 yaw_debug_speed_KP = 100.0, yaw_debug_speed_KI = 0.0, yaw_debug_speed_KD = 0.0, yaw_debug_angle_KP = 13.0, yaw_debug_angle_KI = 0.1, yaw_debug_angle_KD = 0.0;
    fp32 pit_debug_speed_KP = 230.0, pit_debug_speed_KI = 0.3, pit_debug_speed_KD = 0.0, pit_debug_angle_KP = 5.5, pit_debug_angle_KI = 0.0, pit_debug_angle_KD = 0.0;
#endif
#if gimbal_debug_start
static void gimbal_debug_mode(Gimbal_Control_t *gimbal_motor)
{
    YAW_ABS_CTRL 	=	0;
    PITCH_ABS_CTRL 	=	0;

    gimbal_pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_angle_pid,
                     yaw_debug_angle_KP, yaw_debug_angle_KI, yaw_debug_angle_KD);

    pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_gyro_pid,
              yaw_debug_speed_KP, yaw_debug_speed_KI, yaw_debug_speed_KD);

    gimbal_pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_angle_pid,
                     pit_debug_angle_KP, pit_debug_angle_KI, pit_debug_angle_KD);

    pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_gyro_pid,
              pit_debug_speed_KP, pit_debug_speed_KI, pit_debug_speed_KD);

    if(gimbal_motor->gimbal_monitor_point[TX2DataWGD].errorExist == 0)
    {
        gimbal_motor->gimbal_pitch_motor.gimbal_angle_set =  //可以在自瞄时手动调整云台位置
            -gimbal_motor->pitch_angle_PcCtrl_ref + gimbal_motor->gimbal_pitch_motor.relative_angle;

        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set =  //可以在自瞄时手动调整云台位置
            gimbal_motor->yaw_angle_PcCtrl_ref + gimbal_motor->gimbal_yaw_motor.relative_angle;
    }

    if(gimbal_motor->gimbal_monitor_point[TX2DataWGD].errorExist == 1)
    {
        gimbal_motor->gimbal_pitch_motor.gimbal_angle_set = gimbal_motor->pitch_angle_dynamic_ref;
        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = gimbal_motor->yaw_angle_dynamic_ref;
    }

    if(YAW_ABS_CTRL)
    {
        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = fp32_constrain(gimbal_motor->gimbal_yaw_motor.gimbal_angle_set,
                gimbal_motor->gimbal_yaw_motor.absolute_angle - 15.0f,
                gimbal_motor->gimbal_yaw_motor.absolute_angle + 15.0f);
        gimbal_absolute_angle_control(&gimbal_motor->gimbal_yaw_motor);
    }
    else
    {
        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = fp32_constrain(	gimbal_motor->gimbal_yaw_motor.gimbal_angle_set,
                gimbal_motor->gimbal_yaw_motor.absolute_angle - 15.0f,
                gimbal_motor->gimbal_yaw_motor.absolute_angle + 15.0f);
        gimbal_relative_angle_control(&gimbal_motor->gimbal_yaw_motor);
    }

    if(PITCH_ABS_CTRL)
    {
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
        gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
    else
    {
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
        gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
}

#endif
//float time=0.01;
u8 RFlag_state = 0, GFlag_state = 0,ZFlag_state=0,XFlag_state=0,CFlag_state=0;
kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
//	.B_data = {0.0001125,0.015},
  .A_data = {1, 0.015/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}//500 1000
};//初始化yaw的部分kalman参数
kalman_filter_t yaw_kalman_filter;
/****************************************** GIMBAL  _  MODE  _  CONTRAL *************************************************************************************************/

void GIMBAL_task(void *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    GIMBAL_Init(&gimbal_control);//云台初始化
    //判断电机是否都上线
//    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE))
//    {
    vTaskDelay(GIMBAL_CONTROL_TIME);
    GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈

//    }
    while(1)
    {

        gimbal_behavour_set(&gimbal_control, &INPUTMOD); //云台行为状态机设置
        GIMBAL_Feedback_Update(&gimbal_control);//云台数据反馈
        GIMBAL_Behaviour_update(&gimbal_control); //云台状态更新
        GIMBAL_Mode_Change_Control_Transit(&gimbal_control);//云台状态切换保存，用于状态切换过渡
//#if YAW_TURN
//        Yaw_Can_Set_Current = -gimbal_control.gimbal_yaw_motor.given_current;
//#else
        Yaw_Can_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
//#endif

//#if PITCH_TURN
//        Pitch_Can_Set_Current = -gimbal_control.gimbal_pitch_motor.given_current;
//#else
        Pitch_Can_Set_Current = gimbal_control.gimbal_pitch_motor.given_current;

//#endif
//电机未掉线 检测遥控是否掉线 如果掉线云台停止
        if (!(toe_is_error(YawGimbalMotorTOE) && toe_is_error(PitchGimbalMotorTOE)))
        {
            if (/*toe_is_error(DBUSTOE) || */gimbal_control.gimbal_behaviour ==  GIMBAL_ZERO_FORCE)
            {
                CAN_CMD_GIMBAL(0, 0, 0, 0);
            }
            else
            {
                CAN_CMD_GIMBAL(Pitch_Can_Set_Current, Yaw_Can_Set_Current, 0, 0);
            }
        }

//        if(fabs(gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid.err) < 1.0f && work_turn && FFlag_state == 0)
//        {
//            work_turn = 0;
//            type_turn = 0;
//        }

        vTaskDelay(1);
        #if INCLUDE_uxTaskGetStackHighWaterMark
        GimbalTaskStack = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}

//云台初始化
float rate_kalman_R=100;
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //陀螺仪数据指针获取
    gimbal_init->gimbal_angle_gyro_point = get_Gyro_Angle_Point();
    //电机数据指针获取
    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_yaw_motor.gimbal_encoder_measure = get_Pitch_Gimbal_Encoder_Measure_Point();
    gimbal_init->gimbal_pitch_motor.gimbal_encoder_measure = get_Yaw_Gimbal_Encoder_Measure_Point();

    //卡尔曼滤波器初始化
    /*PID角度误差卡尔曼,一阶*/
    KalmanCreate(&gimbal_init->gimbal_pitch_motor.Error_Kalman, 50, 20);
    KalmanCreate(&gimbal_init->gimbal_yaw_motor.Error_Kalman, 50, 15);
		
	//初始化卡尔曼参数
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);

//		KalmanCreate(&kalman_speed,1,rate_kalman_R);
    KalmanCreate(&kalman_yaw,1,50);
		KalmanCreate(&kalman_pitch,1,50);
    //遥控器数据指针获取
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
    //PC数据指针获取
    gimbal_init->PC_Ctrl_Measure_Point = get_PC_Ctrl_Measure_Point();
    //监测系统指针获取
    gimbal_init->gimbal_monitor_point = getErrorListPoint();
    //获取裁判系统数据指针
    gimbal_init->gimbal_status_measure = get_game_robot_state_t();
    //初始化电机模式
    gimbal_init->gimbal_behaviour = GIMBAL_ZERO_FORCE;
    //归中初始化
    gimbal_control.gimbal_pitch_motor.offset_ecd = Glimbal_Pitch_Offset;
    gimbal_control.gimbal_yaw_motor.offset_ecd = Glimbal_Yaw_Offset;
    //yaw角度环
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_PID_MAX_OUT, YAW_PID_MAX_IOUT, YAW_INIT_PID_KP, YAW_INIT_PID_KI, YAW_INIT_PID_KD);
    //yaw速度环
    PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //pit角度环
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_PID_MAX_OUT, PITCH_PID_MAX_IOUT, PITCH_INIT_PID_KP, PITCH_INIT_PID_KI, PITCH_INIT_PID_KD);
    //pit速度环
    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
}
//底盘无力模式
static void gimbal_zero_force(Gimbal_Control_t *gimbal_motor)
{
    gimbal_motor->gimbal_yaw_motor.given_current = 0;
    gimbal_motor->gimbal_pitch_motor.given_current = 0;
}
//云台初始化
static void gimbal_behaviour_init(Gimbal_Control_t *gimbal_motor)
{
//    TIM_SetCompare1(TIM5, PWM); //默认关闭弹舱

    gimbal_pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_INIT_PID_KP, YAW_INIT_PID_KI, YAW_INIT_PID_KD);
    pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD);

    gimbal_pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_INIT_PID_KP, PITCH_INIT_PID_KI, PITCH_INIT_PID_KD);
    pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD);

    //平缓地让云台移动到中间,防止刚上电狂甩
    gimbal_motor->gimbal_pitch_motor.gimbal_angle_set = RAMP_float(0.0f, gimbal_motor->gimbal_pitch_motor.relative_angle, 6.0f);
    gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
    gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = RAMP_float(0.0f, gimbal_motor->gimbal_yaw_motor.relative_angle, 3.0f);
    gimbal_relative_angle_control(&gimbal_motor->gimbal_yaw_motor);
}
//手控模式

unsigned char mm=0;
static void gimbal_manual_control(Gimbal_Control_t *gimbal_motor)
{
//   	TX_vision_Mes.mode  =	1;//视觉调试
    //采用（陀螺 1/机械 0）控制
    YAW_ABS_CTRL 	=	1;
    PITCH_ABS_CTRL 	=	1;
    TX_vision_Mes.mode  =	0;//发送PC辅助射击模式

    gimbal_motor->gimbal_pitch_motor.gimbal_angle_set = gimbal_motor->pitch_angle_dynamic_ref;
    gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = gimbal_motor->yaw_angle_dynamic_ref;

    if(YAW_ABS_CTRL)
    {
        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = fp32_constrain(gimbal_motor->gimbal_yaw_motor.gimbal_angle_set, gimbal_motor->gimbal_yaw_motor.absolute_angle - 15.0f,
                gimbal_motor->gimbal_yaw_motor.absolute_angle + 15.0f);
        gimbal_absolute_angle_control(&gimbal_motor->gimbal_yaw_motor);
    }
    else
    {
        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = fp32_constrain(gimbal_motor->gimbal_yaw_motor.gimbal_angle_set, gimbal_motor->gimbal_yaw_motor.absolute_angle - 15.0f,
                gimbal_motor->gimbal_yaw_motor.absolute_angle + 15.0f);
        gimbal_relative_angle_control(&gimbal_motor->gimbal_yaw_motor);
    }

    if(PITCH_ABS_CTRL)
    {
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);//pitch轴软件限幅
        gimbal_absolute_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
    else
    {
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
        gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
}
//普通自瞄装甲板
float pitch_rate=0.005f,yaw_rate=0.008f;
int mmmm=0;
int Offline_time=0;
float *yaw_kf_result,finnal_angle;
float t1=0.0f,t2=0.0f,speed_limt=6,yaw_rate_speed=0.020f,for_rate=0.0003f;//t2=1        yaw_rate_speed=0.02
static void gimbal_track_armor(Gimbal_Control_t *gimbal_motor)
{
    //采用（陀螺 1/机械 0）控制
    YAW_ABS_CTRL 	=	1;
    PITCH_ABS_CTRL 	=	1;
    TX_vision_Mes.mode  =	0;//发送PC辅助射击模式

    if(gimbal_motor->gimbal_monitor_point[TX2DataWGD].errorExist == 0)
    {
			Offline_time=0;
//        gimbal_motor->gimbal_pitch_motor.gimbal_angle_set -= PITCH_TURN * (gimbal_motor->pitch_angle_PcCtrl_ref * pitch_rate + gimbal_motor->gimbal_rc_ctrl->rc.ch[PitchChannel]
//                * STICK_TO_PITCH_ANGLE_INC_FACT + fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.y, -45, +45)
//                * MOUSE_TO_PITCH_ANGLE_INC_FACT);
//        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set   += gimbal_motor->yaw_angle_PcCtrl_ref * yaw_rate - gimbal_motor->gimbal_rc_ctrl->rc.ch[YawChannel]
//                * STICK_TO_YAW_ANGLE_INC_FACT - fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.x, -45, +45)
//                * MOUSE_TO_YAW_ANGLE_INC_FACT;
					if(Forecast_or_not==1)//判断视觉视觉数据是否更新
			{	
				Forecast_or_not=0;
			forecast_auxiliary_aim.now_time =xTaskGetTickCount();
			forecast_auxiliary_aim.time_err =forecast_auxiliary_aim.now_time -forecast_auxiliary_aim.last_time ;//计算两次时间差
			forecast_auxiliary_aim.last_time =forecast_auxiliary_aim.now_time ;
      forecast_auxiliary_aim.pitch_angle =gimbal_motor->pitch_angle_PcCtrl_ref ;
			forecast_auxiliary_aim.pitch_speed =(forecast_auxiliary_aim.pitch_angle-forecast_auxiliary_aim.last_pitch_angle)/forecast_auxiliary_aim.time_err*1000.0f;//计算速度
			forecast_auxiliary_aim.last_pitch_angle=forecast_auxiliary_aim.pitch_angle;
			forecast_auxiliary_aim.pitch_forecast_angle=forecast_auxiliary_aim.pitch_speed*t1;//
			forecast_auxiliary_aim.yaw_angle=gimbal_motor->yaw_angle_PcCtrl_ref ;
//			forecast_auxiliary_aim.yaw_speed =(forecast_auxiliary_aim.yaw_angle-forecast_auxiliary_aim.last_yaw_angle )/forecast_auxiliary_aim.time_err*1000.0f;
//			forecast_auxiliary_aim.last_yaw_angle=forecast_auxiliary_aim.yaw_angle;
			forecast_auxiliary_aim.yaw_speed=gimbal_motor->gimbal_angle_gyro_point->V_Z;
			forecast_auxiliary_aim.yaw_forecast_angle=gimbal_motor->gimbal_yaw_motor .absolute_angle +gimbal_motor->yaw_angle_PcCtrl_ref ;
//			forecast_auxiliary_aim.yaw_speed=gimbal_motor->gimbal_yaw_motor .gimbal_motor_measure ->speed_rpm;
      yaw_kf_result=kalman_filter_calc(&yaw_kalman_filter,forecast_auxiliary_aim.yaw_forecast_angle,forecast_auxiliary_aim.yaw_speed);//卡尔曼状态融合
			forecast_auxiliary_aim.yaw_speed =yaw_kalman_filter.filtered_value[1];//卡尔曼融合之后得速度
//				 [k]=yaw_kalman_filter.filtered_value[0];
//				abs_limit(&forecast_auxiliary_aim.yaw_speed,speed_limt);
//			forecast_auxiliary_aim.yaw_speed=RAMP_float(forecast_auxiliary_aim.yaw_speed,forecast_auxiliary_aim.last_yaw_speed,yaw_rate_speed);
				finnal_angle=*yaw_kf_result+forecast_auxiliary_aim.yaw_forcast_speed*t2;//
      
//      abs_limit(&forecast_auxiliary_aim.yaw_speed,speed_limt);	
//			forecast_auxiliary_aim.yaw_speed=RAMP_float(forecast_auxiliary_aim.yaw_speed,forecast_auxiliary_aim.last_yaw_speed,yaw_rate_speed);//对速度变化做斜坡处理	     
//       forecast_auxiliary_aim.yaw_speed=KalmanFilter(&kalman_speed,forecast_auxiliary_aim.yaw_speed);//观测电机转数变化使用				k++;if(k==30)	k=0;
			
			forecast_auxiliary_aim.last_yaw_speed =	forecast_auxiliary_aim.yaw_speed;
			}
	/*************************************************************************************/		
			
			gimbal_motor->pitch_angle_PcCtrl_ref=	KalmanFilter(&kalman_pitch,gimbal_motor->pitch_angle_PcCtrl_ref);//数据滤波
			gimbal_motor->PC_RAM_pitch_ref =RAMP_float(gimbal_motor->pitch_angle_PcCtrl_ref,0,10);		
			gimbal_motor->gimbal_pitch_motor.gimbal_angle_set=(-gimbal_motor->gimbal_pitch_motor.absolute_angle-gimbal_motor->PC_RAM_pitch_ref)*PITCH_TURN;

			gimbal_motor->gimbal_yaw_motor .gimbal_for_angle_set -= fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.x, -45, +45) * /*MOUSE_TO_PITCH_ANGLE_INC_FACT*/for_rate;
   
			gimbal_motor->yaw_angle_PcCtrl_ref=	KalmanFilter(&kalman_yaw,gimbal_motor->yaw_angle_PcCtrl_ref);//数据滤波
			gimbal_motor->PC_RAM_yaw_ref =RAMP_float(gimbal_motor->yaw_angle_PcCtrl_ref,0,10);	
			gimbal_motor->gimbal_yaw_motor .gimbal_angle_set=/*finnal_angle*/gimbal_motor->gimbal_yaw_motor .absolute_angle +gimbal_motor->PC_RAM_yaw_ref+gimbal_motor->gimbal_yaw_motor .gimbal_for_angle_set/*+forecast_auxiliary_aim.yaw_forecast_angle*/;
    }

    if(gimbal_motor->gimbal_monitor_point[TX2DataWGD].errorExist == 1)//TX2离线
    {
						Offline_time++;
			if(Offline_time>100)
			{gimbal_motor->gimbal_yaw_motor .gimbal_for_angle_set=0;}
        gimbal_motor->gimbal_pitch_motor.gimbal_angle_set -= PITCH_TURN * (gimbal_motor->gimbal_rc_ctrl->rc.ch[PitchChannel] * STICK_TO_PITCH_ANGLE_INC_FACT
                + fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.y, -45, +45) * MOUSE_TO_PITCH_ANGLE_INC_FACT);
        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set += (-gimbal_motor->gimbal_rc_ctrl->rc.ch[YawChannel] * STICK_TO_YAW_ANGLE_INC_FACT)
                - (fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.x, -45, +45) * MOUSE_TO_YAW_ANGLE_INC_FACT);
//		gimbal_motor->yaw_angle_PcCtrl_ref = 0;
//		gimbal_motor->pitch_angle_PcCtrl_ref = 0;
    }
    

    if(YAW_ABS_CTRL)
    {
        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = fp32_constrain(gimbal_motor->gimbal_yaw_motor.gimbal_angle_set, gimbal_motor->gimbal_yaw_motor.absolute_angle - 15.0f,
                gimbal_motor->gimbal_yaw_motor.absolute_angle + 15.0f);
        gimbal_absolute_angle_control(&gimbal_motor->gimbal_yaw_motor);
    }
    else
    {
        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = fp32_constrain(gimbal_motor->gimbal_yaw_motor.gimbal_angle_set,  gimbal_motor->gimbal_yaw_motor.absolute_angle - 15.0f,
                gimbal_motor->gimbal_yaw_motor.absolute_angle + 15.0f);
        gimbal_relative_angle_control(&gimbal_motor->gimbal_yaw_motor);
    }

    if(PITCH_ABS_CTRL)
    {
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);//pitch轴软件限幅
        gimbal_absolute_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
    else
    {
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
        gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
}
//击打能量机关模式
float yaw_buff_forecast_time=0.0f,pitch_buff_forecast_time=0.0f;

static void gimbal_shoot_buff(Gimbal_Control_t *gimbal_motor)
{
    //采用（陀螺 1/机械 0）控制
    YAW_ABS_CTRL 	=	1;
    PITCH_ABS_CTRL 	=	1;
    TX_vision_Mes.mode  =	1;//发送PC射符模式
	//观察视觉发送数据使用

    if(gimbal_motor->gimbal_monitor_point[TX2DataWGD].errorExist == 0)
    {
////						gimbal_motor->pitch_angle_PcCtrl_ref=	KalmanFilter(&kalman_pitch,gimbal_motor->pitch_angle_PcCtrl_ref);//数据滤波
//			gimbal_motor->PC_RAM_pitch_ref =RAMP_float(gimbal_motor->pitch_angle_PcCtrl_ref,0,10);		
//			gimbal_motor->gimbal_pitch_motor.gimbal_angle_set=(-gimbal_motor->gimbal_pitch_motor.absolute_angle-gimbal_motor->PC_RAM_pitch_ref)*PITCH_TURN;
////			gimbal_motor->yaw_angle_PcCtrl_ref=	KalmanFilter(&kalman_yaw,gimbal_motor->yaw_angle_PcCtrl_ref);//数据滤波
//			gimbal_motor->PC_RAM_yaw_ref =RAMP_float(gimbal_motor->yaw_angle_PcCtrl_ref,0,10);	
//			gimbal_motor->gimbal_yaw_motor .gimbal_angle_set=/*finnal_angle*/gimbal_motor->gimbal_yaw_motor .absolute_angle +gimbal_motor->PC_RAM_yaw_ref/*+forecast_auxiliary_aim.yaw_forecast_angle*/;
//      
			
        gimbal_motor->gimbal_pitch_motor.gimbal_angle_set -= PITCH_TURN * (gimbal_motor->pitch_angle_PcCtrl_ref * pitch_rate
                + gimbal_motor->gimbal_rc_ctrl->rc.ch[PitchChannel] * STICK_TO_PITCH_ANGLE_INC_FACT
                + fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.y, -45, +45) * MOUSE_TO_PITCH_ANGLE_INC_FACT);

        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set += gimbal_motor->yaw_angle_PcCtrl_ref *yaw_rate
                - gimbal_motor->gimbal_rc_ctrl->rc.ch[YawChannel] * STICK_TO_YAW_ANGLE_INC_FACT
                - fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.x, -45, +45) * MOUSE_TO_YAW_ANGLE_INC_FACT;

    }
    else if(gimbal_motor->gimbal_monitor_point[TX2DataWGD].errorExist == 1)
    {

        gimbal_motor->gimbal_pitch_motor.gimbal_angle_set -= PITCH_TURN * (gimbal_motor->gimbal_rc_ctrl->rc.ch[PitchChannel] * STICK_TO_PITCH_ANGLE_INC_FACT
                + fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.y, -45, +45) * MOUSE_TO_PITCH_ANGLE_INC_FACT);
        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set   += (-gimbal_motor->gimbal_rc_ctrl->rc.ch[YawChannel] * STICK_TO_YAW_ANGLE_INC_FACT)
                - (fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.x, -45, +45) * MOUSE_TO_YAW_ANGLE_INC_FACT);
    }

    if(YAW_ABS_CTRL)
    {
        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = fp32_constrain(gimbal_motor->gimbal_yaw_motor.gimbal_angle_set,
                gimbal_motor->gimbal_yaw_motor.absolute_angle - 15.0f,
                gimbal_motor->gimbal_yaw_motor.absolute_angle + 15.0f);
        gimbal_absolute_angle_control(&gimbal_motor->gimbal_yaw_motor);
    }
    else
    {
        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = fp32_constrain(gimbal_motor->gimbal_yaw_motor.gimbal_angle_set,
                gimbal_motor->gimbal_yaw_motor.absolute_angle - 15.0f,
                gimbal_motor->gimbal_yaw_motor.absolute_angle + 15.0f);
        gimbal_relative_angle_control(&gimbal_motor->gimbal_yaw_motor);
    }

    if(PITCH_ABS_CTRL)
    {
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
        gimbal_absolute_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
    else
    {
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
        gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
}
//补弹模式
static void gimbal_loading_bullet(Gimbal_Control_t *gimbal_motor)
{
    gimbal_pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_INIT_PID_KP, YAW_INIT_PID_KI, YAW_INIT_PID_KD);
    pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD);

    gimbal_pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_INIT_PID_KP, PITCH_INIT_PID_KI, PITCH_INIT_PID_KD);
    pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD);

    //斜坡函数——平缓地让云台移动到中间,防止狂甩
    gimbal_motor->gimbal_pitch_motor.gimbal_angle_set = RAMP_float(0.0f, gimbal_motor->gimbal_pitch_motor.relative_angle, 6.0f);
    gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
    gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = RAMP_float(0.0f, gimbal_motor->gimbal_yaw_motor.relative_angle, 6.0f);
    gimbal_relative_angle_control(&gimbal_motor->gimbal_yaw_motor);
}
//扭头
static void gimbal_turn_180(Gimbal_Control_t *gimbal_motor)
{
    gimbal_pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_INIT_PID_KP, YAW_INIT_PID_KI, YAW_INIT_PID_KD);
    pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD);

    gimbal_pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_INIT_PID_KP, PITCH_INIT_PID_KI, PITCH_INIT_PID_KD);
    pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD);

    gimbal_motor->gimbal_pitch_motor.gimbal_angle_set = RAMP_float(0.0f, gimbal_motor->gimbal_pitch_motor.relative_angle, 10.0f);
    gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
	  gimbal_motor->gimbal_yaw_motor.gimbal_angle_set=gimbal_motor->gimbal_yaw_motor.gimbal_turn_around;
	  gimbal_motor->gimbal_yaw_motor.gimbal_angle_set=fp32_constrain(gimbal_motor->gimbal_yaw_motor.gimbal_angle_set, gimbal_motor->gimbal_yaw_motor.absolute_angle - 30.0f,
                gimbal_motor->gimbal_yaw_motor.absolute_angle + 30.0f);
    gimbal_absolute_angle_control(&gimbal_motor->gimbal_yaw_motor);
	if(fabs(gimbal_motor->gimbal_yaw_motor.gimbal_turn_around-gimbal_motor->gimbal_yaw_motor .absolute_angle)<=1.0f)
	{
	   ZFlag_state=0;
		 CFlag_state=0;
		 XFlag_state=0;
	}
}
/**
  * @brief          云台行为状态机设置
  * @author         RM
  * @param[in]      云台数据指针
  * @retval         返回空
  */

static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set, remote_mode_e *inputmode)
{
    static u8 MOU_RIGH_staste = 0, QFlag_state = 0, VFlag_state = 0;

    if (*inputmode == NULL)
    {
        return;
    }

    if(*inputmode == REMOTE_INPUT)
    {
						QFlag_state=0;
					  RFlag_state=0;
					  GFlag_state=0;
					  ZFlag_state=0;       
		      	GFlag_state = 0;
		    	MOU_RIGH_staste=0;
        //开关控制 云台状态
        if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R]) && switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //左下右中
            gimbal_mode_set->gimbal_behaviour =  GIMBAL_INIT;//初始化
        }
        else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R]) && switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //双中
            gimbal_mode_set->gimbal_behaviour = GIMBAL_MANUAL_MODE; //手动控制
        }
        else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R]) && switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            // 右中左上
            gimbal_mode_set->gimbal_behaviour = GIMBAL_MANUAL_MODE; //手动控制
        }
        else
        {
            gimbal_mode_set->gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }
    }

    /*********************************** 按 键 控 制**************************************/
    if(*inputmode == KEYMOUSE_INPUT)
    {
        /* 鼠标数据更新
        	右键开启/关闭 ———— 自瞄模式
        */
        if(!(gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r != 0))
        {
            MOU_RIGH_F = 1;
        }

        if(gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r && MOU_RIGH_F == 1)
        {
            MOU_RIGH_F = 0;
            GFlag_state = 0;
            MOU_RIGH_staste ++;
            MOU_RIGH_staste %= 2;
						QFlag_state=0;
					  RFlag_state=0;
					  GFlag_state=0;
					  ZFlag_state=0;
        }

        if(MOU_RIGH_staste)
        {
            gimbal_mode_set->gimbal_behaviour = GIMBAL_TRACK_ARMOR;
        }

        /* 键盘数据更新
        	G  取消/启动 ———— 射符模式
        */
        if(!((gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G) != 0))
        {
            G_Flag = 1;
        }

        if(gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G && G_Flag == 1)
        {
            G_Flag = 0;
            MOU_RIGH_staste = 0;
            GFlag_state++;
            GFlag_state %= 2;
						QFlag_state=0;
					  RFlag_state=0;
					  ZFlag_state=0;
        }

        if(GFlag_state)
        {
            gimbal_mode_set->gimbal_behaviour = GIMBAL_SHOOT_BUFF;
        }

//			R 打开/关闭——弹舱
        if(!((gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_R) != 0))
        {
            R_Flag = 1;
        }

        if(gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_R && R_Flag == 1)
        {
            R_Flag = 0;
            RFlag_state++;
            RFlag_state %= 2;
						QFlag_state=0;
					  ZFlag_state=0;
					  GFlag_state=0;
					  MOU_RIGH_staste=0;
					  FFlag_state=0;
        }

        if(RFlag_state == 0)
        {
            TIM_SetCompare1(TIM5, PWM);
        }

        if(RFlag_state)
        {
            gimbal_mode_set->gimbal_behaviour = GIMBAL_LOADING_BULLET;
            TIM_SetCompare1(TIM5, 19000);
        }
				//Z键转头
/***********************************************************************************************************/
//z 180
          if(!((gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Z) != 0))
        {
            Z_Flag = 1;
        }

        if(gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Z && Z_Flag == 1)
        {
            Z_Flag = 0;
            MOU_RIGH_staste = 0;
            ZFlag_state++;
            ZFlag_state %= 2;
					  QFlag_state=0;
					  RFlag_state=0;
					  GFlag_state=0;
					  CFlag_state=0;
					  XFlag_state=0;
						gimbal_mode_set->gimbal_yaw_motor.gimbal_turn_around=gimbal_mode_set->gimbal_yaw_motor .absolute_angle+180; 
        }

        if(ZFlag_state)
        {
            gimbal_mode_set->gimbal_behaviour = GIMBAL_TURN180;
        }
/********************************************************************************************************************/
//x  90
	          if(!((gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X) != 0))
        {
            X_Flag = 1;
        }

        if(gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X && X_Flag == 1)
        {
            X_Flag = 0;
            MOU_RIGH_staste = 0;
            XFlag_state++;
            XFlag_state %= 2;
					  QFlag_state=0;
					  RFlag_state=0;
					  GFlag_state=0;
					  CFlag_state=0;
					  ZFlag_state=0;
						gimbal_mode_set->gimbal_yaw_motor.gimbal_turn_around=gimbal_mode_set->gimbal_yaw_motor .absolute_angle+90; 
        }

        if(XFlag_state)
        {
            gimbal_mode_set->gimbal_behaviour = GIMBAL_TURN180;
        }	
/**********************************************************************************************************************/
	          if(!((gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C) != 0))
        {
            C_Flag = 1;
        }

        if(gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C && C_Flag == 1)
        {
            C_Flag = 0;
            MOU_RIGH_staste = 0;
            CFlag_state++;
            CFlag_state %= 2;
					  QFlag_state=0;
					  RFlag_state=0;
					  GFlag_state=0;
					  XFlag_state=0;
						gimbal_mode_set->gimbal_yaw_motor.gimbal_turn_around=gimbal_mode_set->gimbal_yaw_motor .absolute_angle-90; 
        }

        if(CFlag_state)
        {
            gimbal_mode_set->gimbal_behaviour = GIMBAL_TURN180;
        }				
        /* 键盘数据更新
        	无特殊模式启动时 ———— 进入手控云台模式
        */
        if(VFlag_state == 0 && ZFlag_state == 0 && MOU_RIGH_staste == 0 && QFlag_state == 0 && RFlag_state == 0 && GFlag_state == 0 && XFlag_state == 0&& CFlag_state == 0)
        {
            gimbal_mode_set->gimbal_behaviour = GIMBAL_MANUAL_MODE;
        }

//        if(IF_KEY_PRESSED_Z && type_turn == 0 /*&& FFlag_state == 0 */&& work_turn == 0)
//        {
//            work_turn = 1;
//            type_turn = 1;
//        }
    }

    if(*inputmode  == RUN_STOP)//(双下)停止指令
    {
        #if gimbal_debug_start

        if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R])
                && switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //左上右下
            gimbal_mode_set->gimbal_behaviour = GIMBAL_DEBUG_MODE;
        }

        #else

        if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R])
                && switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //左上右下

        }

        #endif

        if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R]) && switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            gimbal_mode_set->gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }
    }
}
/*
  * @brief          云台状态切换保存，用于状态切换过渡
  * @author         RM
  * @param[in]      云台数据指针
  * @retval         返回空
*/
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }

    if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_ZERO_FORCE && gimbal_mode_change->gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        //掉电
        gimbal_mode_change->gimbal_yaw_motor.given_current = 0;
        gimbal_mode_change->gimbal_pitch_motor.given_current = 0;
    }
    else if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_INIT && gimbal_mode_change->gimbal_behaviour == GIMBAL_INIT)
    {
        //初始化
        gimbal_mode_change->pitch_angle_dynamic_ref = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->yaw_angle_dynamic_ref = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			gimbal_mode_change->gimbal_yaw_motor .gimbal_for_angle_set=0;
    }
    else if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_MANUAL_MODE && gimbal_mode_change->gimbal_behaviour == GIMBAL_MANUAL_MODE)
    {
        //手控
        gimbal_pid_reset(&gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_MANUAL_ANGLE_PID_KP, YAW_MANUAL_ANGLE_PID_KI, YAW_MANUAL_ANGLE_PID_KD);
        pid_reset(&gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_MANUAL_SPEED_PID_KP, YAW_MANUAL_SPEED_PID_KI, YAW_MANUAL_SPEED_PID_KD);
        gimbal_pid_reset(&gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_MANUAL_ANGLE_PID_KP, PITCH_MANUAL_ANGLE_PID_KI, PITCH_MANUAL_ANGLE_PID_KD);
        pid_reset(&gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_MANUAL_SPEED_PID_KP, PITCH_MANUAL_SPEED_PID_KI, PITCH_MANUAL_SPEED_PID_KD);

        gimbal_mode_change->pitch_angle_dynamic_ref = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->yaw_angle_dynamic_ref = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			gimbal_mode_change->gimbal_yaw_motor .gimbal_for_angle_set=0;
    }
    else if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_TRACK_ARMOR && gimbal_mode_change->gimbal_behaviour == GIMBAL_TRACK_ARMOR)
    {
        //自瞄
        gimbal_pid_reset(&gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_TRACK_ANGLE_PID_KP, YAW_TRACK_ANGLE_PID_KI, YAW_TRACK_ANGLE_PID_KD);
        pid_reset(&gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_TRACK_SPEED_PID_KP, YAW_TRACK_SPEED_PID_KI, YAW_TRACK_SPEED_PID_KD);
        gimbal_pid_reset(&gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_TRACK_ANGLE_PID_KP, PITCH_TRACK_ANGLE_PID_KI, PITCH_TRACK_ANGLE_PID_KD);
        pid_reset(&gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_TRACK_SPEED_PID_KP, PITCH_TRACK_SPEED_PID_KI, PITCH_TRACK_SPEED_PID_KD);

        gimbal_mode_change->pitch_angle_dynamic_ref = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->yaw_angle_dynamic_ref = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }

    else if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_SHOOT_BUFF && gimbal_mode_change->gimbal_behaviour == GIMBAL_SHOOT_BUFF)
    {
        //射符
			gimbal_mode_change->gimbal_yaw_motor .gimbal_for_angle_set=0;
        gimbal_pid_reset(&gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_STBUF_ANGLE_PID_KP, YAW_STBUF_ANGLE_PID_KI, YAW_STBUF_ANGLE_PID_KD);
        pid_reset(&gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_STBUF_SPEED_PID_KP, YAW_STBUF_SPEED_PID_KI, YAW_STBUF_SPEED_PID_KD);
        gimbal_pid_reset(&gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_STBUF_ANGLE_PID_KP, PITCH_STBUF_ANGLE_PID_KI, PITCH_STBUF_ANGLE_PID_KD);
        pid_reset(&gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_STBUF_SPEED_PID_KP, PITCH_STBUF_SPEED_PID_KI, PITCH_STBUF_SPEED_PID_KD);

        gimbal_mode_change->pitch_angle_dynamic_ref = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->yaw_angle_dynamic_ref = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_LOADING_BULLET && gimbal_mode_change->gimbal_behaviour == GIMBAL_LOADING_BULLET)
    {
        //初始化
			gimbal_mode_change->gimbal_yaw_motor .gimbal_for_angle_set=0;
        gimbal_mode_change->pitch_angle_dynamic_ref = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->yaw_angle_dynamic_ref = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }

	else if(gimbal_mode_change->last_gimbal_behaviour!=GIMBAL_TURN180 && gimbal_mode_change->gimbal_behaviour==GIMBAL_TURN180)
	{	//转头
		gimbal_mode_change->gimbal_yaw_motor .gimbal_for_angle_set=0;
		gimbal_mode_change->pitch_angle_dynamic_ref = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
		gimbal_mode_change->yaw_angle_dynamic_ref=gimbal_mode_change->gimbal_yaw_motor.absolute_angle;


	}
    gimbal_mode_change->last_gimbal_behaviour = gimbal_mode_change->gimbal_behaviour;
}
//云台数据更新
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }

    static float YAW_MIN = 0.0f, YAW_MAX = 0.0f;
    static float yaw_ecd_ratio = YAW_MOTO_POSITIVE_DIR * YAW_DECELE_RATIO / ENCODER_ANGLE_RATIO;
    static float pit_ecd_ratio = PIT_MOTO_POSITIVE_DIR * PIT_DECELE_RATIO / ENCODER_ANGLE_RATIO;

    if(YAW_ABS_CTRL)
    {
        //yaw 不同控制方式 切换不同方式限幅，防止期望过大疯转
        YAW_MIN = gimbal_feedback_update->gimbal_yaw_motor.absolute_angle - 15.0f;
        YAW_MAX = gimbal_feedback_update->gimbal_yaw_motor.absolute_angle + 15.0f;
    }
    else
    {
        YAW_MIN = gimbal_feedback_update->gimbal_yaw_motor.relative_angle - 15.0f;
        YAW_MAX = gimbal_feedback_update->gimbal_yaw_motor.relative_angle + 15.0f;
    }

    //鼠标输入限幅
    int16_valimit(gimbal_feedback_update->gimbal_rc_ctrl->mouse.x, -45, +45);
    int16_valimit(gimbal_feedback_update->gimbal_rc_ctrl->mouse.y, -45, +45);
    //云台数据更新
    gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = PITCH_TURN * (gimbal_feedback_update->gimbal_angle_gyro_point->ROLL);//陀螺仪ROLL角度
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = PITCH_TURN * (gimbal_feedback_update->gimbal_angle_gyro_point->V_X);//陀螺仪加速度
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = pit_ecd_ratio * get_relative_pos(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,	gimbal_feedback_update->gimbal_pitch_motor.offset_ecd); //编码器角度
    re_pitch_relative_angle = gimbal_feedback_update->gimbal_pitch_motor.relative_angle;

    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = -(gimbal_feedback_update->gimbal_angle_gyro_point->YAW);//陀螺仪yaw角度
    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = (gimbal_feedback_update->gimbal_angle_gyro_point->V_Z);//陀螺仪加速度
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = yaw_ecd_ratio * get_relative_pos(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_feedback_update->gimbal_yaw_motor.offset_ecd);//编码器角度
    VAL_LIMIT(gimbal_feedback_update->gimbal_yaw_motor.absolute_angle, gimbal_feedback_update->gimbal_yaw_motor.absolute_angle - 30, gimbal_feedback_update->gimbal_yaw_motor.absolute_angle + 30);

    re_yaw_absolute_angle = gimbal_feedback_update->gimbal_yaw_motor.absolute_angle;

    //pc动态输入角度更新
    gimbal_feedback_update->pitch_angle_PcCtrl_ref = gimbal_feedback_update->PC_Ctrl_Measure_Point->PcDate.angle_pitch;
    gimbal_feedback_update->yaw_angle_PcCtrl_ref = gimbal_feedback_update->PC_Ctrl_Measure_Point->PcDate.angle_yaw;

    //动态输入角度更新
    gimbal_feedback_update->pitch_angle_dynamic_ref += PITCH_TURN * (gimbal_feedback_update->gimbal_rc_ctrl->rc.ch[PitchChannel] * STICK_TO_PITCH_ANGLE_INC_FACT - fp32_constrain(gimbal_feedback_update->gimbal_rc_ctrl->mouse.y, -45, +45) * MOUSE_TO_PITCH_ANGLE_INC_FACT);//此处鼠标参数负号为取反
    gimbal_feedback_update->yaw_angle_dynamic_ref -=  gimbal_feedback_update->gimbal_rc_ctrl->rc.ch[YawChannel] * STICK_TO_YAW_ANGLE_INC_FACT + fp32_constrain(gimbal_feedback_update->gimbal_rc_ctrl->mouse.x, -45, +45) * MOUSE_TO_YAW_ANGLE_INC_FACT;
    //限幅
    VAL_LIMIT(gimbal_feedback_update->pitch_angle_dynamic_ref, PITCH_MIN, PITCH_MAX);
    VAL_LIMIT(gimbal_feedback_update->yaw_angle_dynamic_ref, YAW_MIN, YAW_MAX); //yaw 不同控制方式 切换不同方式限幅，防止期望过大疯转

}
//云台状态更新
static void GIMBAL_Behaviour_update(Gimbal_Control_t *gimbal_Behaviour)
{
    if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_ZERO_FORCE)//掉电离线
    {
        gimbal_zero_force(gimbal_Behaviour);
    }

    else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_INIT)//归中模式
    {
        gimbal_behaviour_init(gimbal_Behaviour);
    }

   else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_MANUAL_MODE)//手动控制
    {
        gimbal_manual_control(gimbal_Behaviour);
    }

   else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_TRACK_ARMOR)//自瞄
    {
        gimbal_track_armor(gimbal_Behaviour);
    }

    else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_SHOOT_BUFF)//能量机关
    {
        gimbal_shoot_buff(gimbal_Behaviour);
    }

   else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_LOADING_BULLET)//补弹模式
    {
        gimbal_loading_bullet(gimbal_Behaviour);
    }

   else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_TURN180)//扭头
    {
        gimbal_turn_180(gimbal_Behaviour);
    }
    else 
		{
		 gimbal_behaviour_init(gimbal_Behaviour);
		}
    #if gimbal_debug_start

    if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_DEBUG_MODE)
    {
        gimbal_debug_mode(gimbal_Behaviour);
    }

    #endif
}
/*********************************************************************************************************************************************************************/
//使用电机相对角度控制
static void gimbal_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_angle_pid, gimbal_motor->relative_angle, gimbal_motor->gimbal_angle_set, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->Error_Kalman);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
//使用电机绝对角度控制电机
static void gimbal_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->gimbal_angle_set, gimbal_motor->motor_gyro, gimbal_motor->Error_Kalman);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
//得到电机相对角度
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;

    if (center_offset >= 4096)
    {
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else
    {
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }

    return tmp;
}

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, extKalman_t Kal_Gim)
{
    fp32 err;
//    fp32 handle_err;

    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->get = get;
    pid->set = set;

    err = set - get;
    err = KalmanFilter(&Kal_Gim, err);//卡尔曼处理角度误差
    pid->err = /*handle_err*/err;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

static void gimbal_pid_reset(Gimbal_PID_t *pid, float p, float i, float d)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = p;
    pid->ki = i;
    pid->kd = d;

    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->out  = 0;
}
/*********************************************************************************************************************************************************************/
const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

