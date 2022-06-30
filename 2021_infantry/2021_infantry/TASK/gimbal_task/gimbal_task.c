/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note       Ԥ���룺��������ֵ��PID��
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020     		RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */
#include "gimbal_task.h"

static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);//��̨��ʼ��
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);//��̨���ݸ���
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set, remote_mode_e *inputmode); //��̨��Ϊ����
static void GIMBAL_Behaviour_update(Gimbal_Control_t *gimbal_Behaviour);//��̨��Ϊ����
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);//�õ���ԽǶ�
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);//��̨״̬�л����棬����״̬�л�����
static void gimbal_zero_force(Gimbal_Control_t *gimbal_motor);//����ģʽ
static void gimbal_behaviour_init(Gimbal_Control_t *gimbal_motor);//��̨��ʼ��ģʽ
static void gimbal_manual_control(Gimbal_Control_t *gimbal_motor);//�ֶ�����ģʽ
static void gimbal_track_armor(Gimbal_Control_t *gimbal_motor);//����ģʽ
static void gimbal_shoot_buff(Gimbal_Control_t *gimbal_motor);//���ģʽ
static void gimbal_loading_bullet(Gimbal_Control_t *gimbal_motor);//װ��ģʽ
static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);//��̨PID��ʼ��
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta, extKalman_t Gim); //PID����
static void gimbal_pid_reset(Gimbal_PID_t *pid, float p, float i, float d);//PID����
static void gimbal_relative_angle_control(Gimbal_Motor_t *gimbal_motor);//��̨��ԽǶȿ���
static void gimbal_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);//��̨���ԽǶȿ���
float re_yaw_absolute_angle;
float re_pitch_relative_angle;
Forecast  forecast_auxiliary_aim;
Forecast  forecast_auxiliary_buff;
#if gimbal_debug_start
    static void gimbal_debug_mode(Gimbal_Control_t *gimbal_motor);
#endif
//��̨���������������
Gimbal_Control_t gimbal_control;//static
//ң������ģʽ�趨
extern unsigned char  Forecast_or_not;
extern remote_mode_e INPUTMOD;
uint8_t Z_Flag = 0,X_Flag=0,C_Flag=0;
u8 RFlag_state;
//���͵�can ָ��
int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0;//static

//�Ƿ�ʹ�������ǿ���
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
#if gimbal_debug_start //debugģʽ
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
        gimbal_motor->gimbal_pitch_motor.gimbal_angle_set =  //����������ʱ�ֶ�������̨λ��
            -gimbal_motor->pitch_angle_PcCtrl_ref + gimbal_motor->gimbal_pitch_motor.relative_angle;

        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set =  //����������ʱ�ֶ�������̨λ��
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
  .A_data = {1, 0.015/*0.001*/, 0, 1},//����ʱ����
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}//500 1000
};//��ʼ��yaw�Ĳ���kalman����
kalman_filter_t yaw_kalman_filter;
/****************************************** GIMBAL  _  MODE  _  CONTRAL *************************************************************************************************/

void GIMBAL_task(void *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    GIMBAL_Init(&gimbal_control);//��̨��ʼ��
    //�жϵ���Ƿ�����
//    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE))
//    {
    vTaskDelay(GIMBAL_CONTROL_TIME);
    GIMBAL_Feedback_Update(&gimbal_control);             //��̨���ݷ���

//    }
    while(1)
    {

        gimbal_behavour_set(&gimbal_control, &INPUTMOD); //��̨��Ϊ״̬������
        GIMBAL_Feedback_Update(&gimbal_control);//��̨���ݷ���
        GIMBAL_Behaviour_update(&gimbal_control); //��̨״̬����
        GIMBAL_Mode_Change_Control_Transit(&gimbal_control);//��̨״̬�л����棬����״̬�л�����
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
//���δ���� ���ң���Ƿ���� ���������ֹ̨ͣ
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

//��̨��ʼ��
float rate_kalman_R=100;
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //����������ָ���ȡ
    gimbal_init->gimbal_angle_gyro_point = get_Gyro_Angle_Point();
    //�������ָ���ȡ
    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_yaw_motor.gimbal_encoder_measure = get_Pitch_Gimbal_Encoder_Measure_Point();
    gimbal_init->gimbal_pitch_motor.gimbal_encoder_measure = get_Yaw_Gimbal_Encoder_Measure_Point();

    //�������˲�����ʼ��
    /*PID�Ƕ�������,һ��*/
    KalmanCreate(&gimbal_init->gimbal_pitch_motor.Error_Kalman, 50, 20);
    KalmanCreate(&gimbal_init->gimbal_yaw_motor.Error_Kalman, 50, 15);
		
	//��ʼ������������
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);

//		KalmanCreate(&kalman_speed,1,rate_kalman_R);
    KalmanCreate(&kalman_yaw,1,50);
		KalmanCreate(&kalman_pitch,1,50);
    //ң��������ָ���ȡ
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
    //PC����ָ���ȡ
    gimbal_init->PC_Ctrl_Measure_Point = get_PC_Ctrl_Measure_Point();
    //���ϵͳָ���ȡ
    gimbal_init->gimbal_monitor_point = getErrorListPoint();
    //��ȡ����ϵͳ����ָ��
    gimbal_init->gimbal_status_measure = get_game_robot_state_t();
    //��ʼ�����ģʽ
    gimbal_init->gimbal_behaviour = GIMBAL_ZERO_FORCE;
    //���г�ʼ��
    gimbal_control.gimbal_pitch_motor.offset_ecd = Glimbal_Pitch_Offset;
    gimbal_control.gimbal_yaw_motor.offset_ecd = Glimbal_Yaw_Offset;
    //yaw�ǶȻ�
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_PID_MAX_OUT, YAW_PID_MAX_IOUT, YAW_INIT_PID_KP, YAW_INIT_PID_KI, YAW_INIT_PID_KD);
    //yaw�ٶȻ�
    PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //pit�ǶȻ�
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_PID_MAX_OUT, PITCH_PID_MAX_IOUT, PITCH_INIT_PID_KP, PITCH_INIT_PID_KI, PITCH_INIT_PID_KD);
    //pit�ٶȻ�
    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
}
//��������ģʽ
static void gimbal_zero_force(Gimbal_Control_t *gimbal_motor)
{
    gimbal_motor->gimbal_yaw_motor.given_current = 0;
    gimbal_motor->gimbal_pitch_motor.given_current = 0;
}
//��̨��ʼ��
static void gimbal_behaviour_init(Gimbal_Control_t *gimbal_motor)
{
//    TIM_SetCompare1(TIM5, PWM); //Ĭ�Ϲرյ���

    gimbal_pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_INIT_PID_KP, YAW_INIT_PID_KI, YAW_INIT_PID_KD);
    pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD);

    gimbal_pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_INIT_PID_KP, PITCH_INIT_PID_KI, PITCH_INIT_PID_KD);
    pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD);

    //ƽ��������̨�ƶ����м�,��ֹ���ϵ��˦
    gimbal_motor->gimbal_pitch_motor.gimbal_angle_set = RAMP_float(0.0f, gimbal_motor->gimbal_pitch_motor.relative_angle, 6.0f);
    gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
    gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = RAMP_float(0.0f, gimbal_motor->gimbal_yaw_motor.relative_angle, 3.0f);
    gimbal_relative_angle_control(&gimbal_motor->gimbal_yaw_motor);
}
//�ֿ�ģʽ

unsigned char mm=0;
static void gimbal_manual_control(Gimbal_Control_t *gimbal_motor)
{
//   	TX_vision_Mes.mode  =	1;//�Ӿ�����
    //���ã����� 1/��е 0������
    YAW_ABS_CTRL 	=	1;
    PITCH_ABS_CTRL 	=	1;
    TX_vision_Mes.mode  =	0;//����PC�������ģʽ

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
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);//pitch������޷�
        gimbal_absolute_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
    else
    {
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
        gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
}
//��ͨ����װ�װ�
float pitch_rate=0.005f,yaw_rate=0.008f;
int mmmm=0;
int Offline_time=0;
float *yaw_kf_result,finnal_angle;
float t1=0.0f,t2=0.0f,speed_limt=6,yaw_rate_speed=0.020f,for_rate=0.0003f;//t2=1        yaw_rate_speed=0.02
static void gimbal_track_armor(Gimbal_Control_t *gimbal_motor)
{
    //���ã����� 1/��е 0������
    YAW_ABS_CTRL 	=	1;
    PITCH_ABS_CTRL 	=	1;
    TX_vision_Mes.mode  =	0;//����PC�������ģʽ

    if(gimbal_motor->gimbal_monitor_point[TX2DataWGD].errorExist == 0)
    {
			Offline_time=0;
//        gimbal_motor->gimbal_pitch_motor.gimbal_angle_set -= PITCH_TURN * (gimbal_motor->pitch_angle_PcCtrl_ref * pitch_rate + gimbal_motor->gimbal_rc_ctrl->rc.ch[PitchChannel]
//                * STICK_TO_PITCH_ANGLE_INC_FACT + fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.y, -45, +45)
//                * MOUSE_TO_PITCH_ANGLE_INC_FACT);
//        gimbal_motor->gimbal_yaw_motor.gimbal_angle_set   += gimbal_motor->yaw_angle_PcCtrl_ref * yaw_rate - gimbal_motor->gimbal_rc_ctrl->rc.ch[YawChannel]
//                * STICK_TO_YAW_ANGLE_INC_FACT - fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.x, -45, +45)
//                * MOUSE_TO_YAW_ANGLE_INC_FACT;
					if(Forecast_or_not==1)//�ж��Ӿ��Ӿ������Ƿ����
			{	
				Forecast_or_not=0;
			forecast_auxiliary_aim.now_time =xTaskGetTickCount();
			forecast_auxiliary_aim.time_err =forecast_auxiliary_aim.now_time -forecast_auxiliary_aim.last_time ;//��������ʱ���
			forecast_auxiliary_aim.last_time =forecast_auxiliary_aim.now_time ;
      forecast_auxiliary_aim.pitch_angle =gimbal_motor->pitch_angle_PcCtrl_ref ;
			forecast_auxiliary_aim.pitch_speed =(forecast_auxiliary_aim.pitch_angle-forecast_auxiliary_aim.last_pitch_angle)/forecast_auxiliary_aim.time_err*1000.0f;//�����ٶ�
			forecast_auxiliary_aim.last_pitch_angle=forecast_auxiliary_aim.pitch_angle;
			forecast_auxiliary_aim.pitch_forecast_angle=forecast_auxiliary_aim.pitch_speed*t1;//
			forecast_auxiliary_aim.yaw_angle=gimbal_motor->yaw_angle_PcCtrl_ref ;
//			forecast_auxiliary_aim.yaw_speed =(forecast_auxiliary_aim.yaw_angle-forecast_auxiliary_aim.last_yaw_angle )/forecast_auxiliary_aim.time_err*1000.0f;
//			forecast_auxiliary_aim.last_yaw_angle=forecast_auxiliary_aim.yaw_angle;
			forecast_auxiliary_aim.yaw_speed=gimbal_motor->gimbal_angle_gyro_point->V_Z;
			forecast_auxiliary_aim.yaw_forecast_angle=gimbal_motor->gimbal_yaw_motor .absolute_angle +gimbal_motor->yaw_angle_PcCtrl_ref ;
//			forecast_auxiliary_aim.yaw_speed=gimbal_motor->gimbal_yaw_motor .gimbal_motor_measure ->speed_rpm;
      yaw_kf_result=kalman_filter_calc(&yaw_kalman_filter,forecast_auxiliary_aim.yaw_forecast_angle,forecast_auxiliary_aim.yaw_speed);//������״̬�ں�
			forecast_auxiliary_aim.yaw_speed =yaw_kalman_filter.filtered_value[1];//�������ں�֮����ٶ�
//				 [k]=yaw_kalman_filter.filtered_value[0];
//				abs_limit(&forecast_auxiliary_aim.yaw_speed,speed_limt);
//			forecast_auxiliary_aim.yaw_speed=RAMP_float(forecast_auxiliary_aim.yaw_speed,forecast_auxiliary_aim.last_yaw_speed,yaw_rate_speed);
				finnal_angle=*yaw_kf_result+forecast_auxiliary_aim.yaw_forcast_speed*t2;//
      
//      abs_limit(&forecast_auxiliary_aim.yaw_speed,speed_limt);	
//			forecast_auxiliary_aim.yaw_speed=RAMP_float(forecast_auxiliary_aim.yaw_speed,forecast_auxiliary_aim.last_yaw_speed,yaw_rate_speed);//���ٶȱ仯��б�´���	     
//       forecast_auxiliary_aim.yaw_speed=KalmanFilter(&kalman_speed,forecast_auxiliary_aim.yaw_speed);//�۲���ת���仯ʹ��				k++;if(k==30)	k=0;
			
			forecast_auxiliary_aim.last_yaw_speed =	forecast_auxiliary_aim.yaw_speed;
			}
	/*************************************************************************************/		
			
			gimbal_motor->pitch_angle_PcCtrl_ref=	KalmanFilter(&kalman_pitch,gimbal_motor->pitch_angle_PcCtrl_ref);//�����˲�
			gimbal_motor->PC_RAM_pitch_ref =RAMP_float(gimbal_motor->pitch_angle_PcCtrl_ref,0,10);		
			gimbal_motor->gimbal_pitch_motor.gimbal_angle_set=(-gimbal_motor->gimbal_pitch_motor.absolute_angle-gimbal_motor->PC_RAM_pitch_ref)*PITCH_TURN;

			gimbal_motor->gimbal_yaw_motor .gimbal_for_angle_set -= fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.x, -45, +45) * /*MOUSE_TO_PITCH_ANGLE_INC_FACT*/for_rate;
   
			gimbal_motor->yaw_angle_PcCtrl_ref=	KalmanFilter(&kalman_yaw,gimbal_motor->yaw_angle_PcCtrl_ref);//�����˲�
			gimbal_motor->PC_RAM_yaw_ref =RAMP_float(gimbal_motor->yaw_angle_PcCtrl_ref,0,10);	
			gimbal_motor->gimbal_yaw_motor .gimbal_angle_set=/*finnal_angle*/gimbal_motor->gimbal_yaw_motor .absolute_angle +gimbal_motor->PC_RAM_yaw_ref+gimbal_motor->gimbal_yaw_motor .gimbal_for_angle_set/*+forecast_auxiliary_aim.yaw_forecast_angle*/;
    }

    if(gimbal_motor->gimbal_monitor_point[TX2DataWGD].errorExist == 1)//TX2����
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
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);//pitch������޷�
        gimbal_absolute_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
    else
    {
        VAL_LIMIT(gimbal_motor->gimbal_pitch_motor.gimbal_angle_set, PITCH_MIN, PITCH_MAX);
        gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
    }
}
//������������ģʽ
float yaw_buff_forecast_time=0.0f,pitch_buff_forecast_time=0.0f;

static void gimbal_shoot_buff(Gimbal_Control_t *gimbal_motor)
{
    //���ã����� 1/��е 0������
    YAW_ABS_CTRL 	=	1;
    PITCH_ABS_CTRL 	=	1;
    TX_vision_Mes.mode  =	1;//����PC���ģʽ
	//�۲��Ӿ���������ʹ��

    if(gimbal_motor->gimbal_monitor_point[TX2DataWGD].errorExist == 0)
    {
////						gimbal_motor->pitch_angle_PcCtrl_ref=	KalmanFilter(&kalman_pitch,gimbal_motor->pitch_angle_PcCtrl_ref);//�����˲�
//			gimbal_motor->PC_RAM_pitch_ref =RAMP_float(gimbal_motor->pitch_angle_PcCtrl_ref,0,10);		
//			gimbal_motor->gimbal_pitch_motor.gimbal_angle_set=(-gimbal_motor->gimbal_pitch_motor.absolute_angle-gimbal_motor->PC_RAM_pitch_ref)*PITCH_TURN;
////			gimbal_motor->yaw_angle_PcCtrl_ref=	KalmanFilter(&kalman_yaw,gimbal_motor->yaw_angle_PcCtrl_ref);//�����˲�
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
//����ģʽ
static void gimbal_loading_bullet(Gimbal_Control_t *gimbal_motor)
{
    gimbal_pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_INIT_PID_KP, YAW_INIT_PID_KI, YAW_INIT_PID_KD);
    pid_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD);

    gimbal_pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_INIT_PID_KP, PITCH_INIT_PID_KI, PITCH_INIT_PID_KD);
    pid_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD);

    //б�º�������ƽ��������̨�ƶ����м�,��ֹ��˦
    gimbal_motor->gimbal_pitch_motor.gimbal_angle_set = RAMP_float(0.0f, gimbal_motor->gimbal_pitch_motor.relative_angle, 6.0f);
    gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
    gimbal_motor->gimbal_yaw_motor.gimbal_angle_set = RAMP_float(0.0f, gimbal_motor->gimbal_yaw_motor.relative_angle, 6.0f);
    gimbal_relative_angle_control(&gimbal_motor->gimbal_yaw_motor);
}
//Ťͷ
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
  * @brief          ��̨��Ϊ״̬������
  * @author         RM
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
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
        //���ؿ��� ��̨״̬
        if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R]) && switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //��������
            gimbal_mode_set->gimbal_behaviour =  GIMBAL_INIT;//��ʼ��
        }
        else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R]) && switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //˫��
            gimbal_mode_set->gimbal_behaviour = GIMBAL_MANUAL_MODE; //�ֶ�����
        }
        else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R]) && switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            // ��������
            gimbal_mode_set->gimbal_behaviour = GIMBAL_MANUAL_MODE; //�ֶ�����
        }
        else
        {
            gimbal_mode_set->gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }
    }

    /*********************************** �� �� �� ��**************************************/
    if(*inputmode == KEYMOUSE_INPUT)
    {
        /* ������ݸ���
        	�Ҽ�����/�ر� �������� ����ģʽ
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

        /* �������ݸ���
        	G  ȡ��/���� �������� ���ģʽ
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

//			R ��/�رա�������
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
				//Z��תͷ
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
        /* �������ݸ���
        	������ģʽ����ʱ �������� �����ֿ���̨ģʽ
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

    if(*inputmode  == RUN_STOP)//(˫��)ָֹͣ��
    {
        #if gimbal_debug_start

        if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R])
                && switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //��������
            gimbal_mode_set->gimbal_behaviour = GIMBAL_DEBUG_MODE;
        }

        #else

        if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R])
                && switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //��������

        }

        #endif

        if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_R]) && switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel_L]))
        {
            gimbal_mode_set->gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }
    }
}
/*
  * @brief          ��̨״̬�л����棬����״̬�л�����
  * @author         RM
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
*/
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }

    if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_ZERO_FORCE && gimbal_mode_change->gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        //����
        gimbal_mode_change->gimbal_yaw_motor.given_current = 0;
        gimbal_mode_change->gimbal_pitch_motor.given_current = 0;
    }
    else if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_INIT && gimbal_mode_change->gimbal_behaviour == GIMBAL_INIT)
    {
        //��ʼ��
        gimbal_mode_change->pitch_angle_dynamic_ref = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->yaw_angle_dynamic_ref = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			gimbal_mode_change->gimbal_yaw_motor .gimbal_for_angle_set=0;
    }
    else if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_MANUAL_MODE && gimbal_mode_change->gimbal_behaviour == GIMBAL_MANUAL_MODE)
    {
        //�ֿ�
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
        //����
        gimbal_pid_reset(&gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_TRACK_ANGLE_PID_KP, YAW_TRACK_ANGLE_PID_KI, YAW_TRACK_ANGLE_PID_KD);
        pid_reset(&gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_TRACK_SPEED_PID_KP, YAW_TRACK_SPEED_PID_KI, YAW_TRACK_SPEED_PID_KD);
        gimbal_pid_reset(&gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_TRACK_ANGLE_PID_KP, PITCH_TRACK_ANGLE_PID_KI, PITCH_TRACK_ANGLE_PID_KD);
        pid_reset(&gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_TRACK_SPEED_PID_KP, PITCH_TRACK_SPEED_PID_KI, PITCH_TRACK_SPEED_PID_KD);

        gimbal_mode_change->pitch_angle_dynamic_ref = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->yaw_angle_dynamic_ref = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }

    else if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_SHOOT_BUFF && gimbal_mode_change->gimbal_behaviour == GIMBAL_SHOOT_BUFF)
    {
        //���
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
        //��ʼ��
			gimbal_mode_change->gimbal_yaw_motor .gimbal_for_angle_set=0;
        gimbal_mode_change->pitch_angle_dynamic_ref = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->yaw_angle_dynamic_ref = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }

	else if(gimbal_mode_change->last_gimbal_behaviour!=GIMBAL_TURN180 && gimbal_mode_change->gimbal_behaviour==GIMBAL_TURN180)
	{	//תͷ
		gimbal_mode_change->gimbal_yaw_motor .gimbal_for_angle_set=0;
		gimbal_mode_change->pitch_angle_dynamic_ref = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
		gimbal_mode_change->yaw_angle_dynamic_ref=gimbal_mode_change->gimbal_yaw_motor.absolute_angle;


	}
    gimbal_mode_change->last_gimbal_behaviour = gimbal_mode_change->gimbal_behaviour;
}
//��̨���ݸ���
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
        //yaw ��ͬ���Ʒ�ʽ �л���ͬ��ʽ�޷�����ֹ���������ת
        YAW_MIN = gimbal_feedback_update->gimbal_yaw_motor.absolute_angle - 15.0f;
        YAW_MAX = gimbal_feedback_update->gimbal_yaw_motor.absolute_angle + 15.0f;
    }
    else
    {
        YAW_MIN = gimbal_feedback_update->gimbal_yaw_motor.relative_angle - 15.0f;
        YAW_MAX = gimbal_feedback_update->gimbal_yaw_motor.relative_angle + 15.0f;
    }

    //��������޷�
    int16_valimit(gimbal_feedback_update->gimbal_rc_ctrl->mouse.x, -45, +45);
    int16_valimit(gimbal_feedback_update->gimbal_rc_ctrl->mouse.y, -45, +45);
    //��̨���ݸ���
    gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = PITCH_TURN * (gimbal_feedback_update->gimbal_angle_gyro_point->ROLL);//������ROLL�Ƕ�
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = PITCH_TURN * (gimbal_feedback_update->gimbal_angle_gyro_point->V_X);//�����Ǽ��ٶ�
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = pit_ecd_ratio * get_relative_pos(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,	gimbal_feedback_update->gimbal_pitch_motor.offset_ecd); //�������Ƕ�
    re_pitch_relative_angle = gimbal_feedback_update->gimbal_pitch_motor.relative_angle;

    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = -(gimbal_feedback_update->gimbal_angle_gyro_point->YAW);//������yaw�Ƕ�
    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = (gimbal_feedback_update->gimbal_angle_gyro_point->V_Z);//�����Ǽ��ٶ�
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = yaw_ecd_ratio * get_relative_pos(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_feedback_update->gimbal_yaw_motor.offset_ecd);//�������Ƕ�
    VAL_LIMIT(gimbal_feedback_update->gimbal_yaw_motor.absolute_angle, gimbal_feedback_update->gimbal_yaw_motor.absolute_angle - 30, gimbal_feedback_update->gimbal_yaw_motor.absolute_angle + 30);

    re_yaw_absolute_angle = gimbal_feedback_update->gimbal_yaw_motor.absolute_angle;

    //pc��̬����Ƕȸ���
    gimbal_feedback_update->pitch_angle_PcCtrl_ref = gimbal_feedback_update->PC_Ctrl_Measure_Point->PcDate.angle_pitch;
    gimbal_feedback_update->yaw_angle_PcCtrl_ref = gimbal_feedback_update->PC_Ctrl_Measure_Point->PcDate.angle_yaw;

    //��̬����Ƕȸ���
    gimbal_feedback_update->pitch_angle_dynamic_ref += PITCH_TURN * (gimbal_feedback_update->gimbal_rc_ctrl->rc.ch[PitchChannel] * STICK_TO_PITCH_ANGLE_INC_FACT - fp32_constrain(gimbal_feedback_update->gimbal_rc_ctrl->mouse.y, -45, +45) * MOUSE_TO_PITCH_ANGLE_INC_FACT);//�˴�����������Ϊȡ��
    gimbal_feedback_update->yaw_angle_dynamic_ref -=  gimbal_feedback_update->gimbal_rc_ctrl->rc.ch[YawChannel] * STICK_TO_YAW_ANGLE_INC_FACT + fp32_constrain(gimbal_feedback_update->gimbal_rc_ctrl->mouse.x, -45, +45) * MOUSE_TO_YAW_ANGLE_INC_FACT;
    //�޷�
    VAL_LIMIT(gimbal_feedback_update->pitch_angle_dynamic_ref, PITCH_MIN, PITCH_MAX);
    VAL_LIMIT(gimbal_feedback_update->yaw_angle_dynamic_ref, YAW_MIN, YAW_MAX); //yaw ��ͬ���Ʒ�ʽ �л���ͬ��ʽ�޷�����ֹ���������ת

}
//��̨״̬����
static void GIMBAL_Behaviour_update(Gimbal_Control_t *gimbal_Behaviour)
{
    if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_ZERO_FORCE)//��������
    {
        gimbal_zero_force(gimbal_Behaviour);
    }

    else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_INIT)//����ģʽ
    {
        gimbal_behaviour_init(gimbal_Behaviour);
    }

   else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_MANUAL_MODE)//�ֶ�����
    {
        gimbal_manual_control(gimbal_Behaviour);
    }

   else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_TRACK_ARMOR)//����
    {
        gimbal_track_armor(gimbal_Behaviour);
    }

    else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_SHOOT_BUFF)//��������
    {
        gimbal_shoot_buff(gimbal_Behaviour);
    }

   else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_LOADING_BULLET)//����ģʽ
    {
        gimbal_loading_bullet(gimbal_Behaviour);
    }

   else if(gimbal_Behaviour->gimbal_behaviour == GIMBAL_TURN180)//Ťͷ
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
//ʹ�õ����ԽǶȿ���
static void gimbal_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_angle_pid, gimbal_motor->relative_angle, gimbal_motor->gimbal_angle_set, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->Error_Kalman);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
//ʹ�õ�����ԽǶȿ��Ƶ��
static void gimbal_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->gimbal_angle_set, gimbal_motor->motor_gyro, gimbal_motor->Error_Kalman);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
//�õ������ԽǶ�
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
    err = KalmanFilter(&Kal_Gim, err);//����������Ƕ����
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

