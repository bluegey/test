#include "chassis_task.h"

//����ʼ����һ��ʱ��
#define 	CHASSIS_TASK_INIT_TIME   100
//ң������ģʽ�趨
extern remote_mode_e INPUTMOD;
extern u8 GFlag_state;
extern u8 ZFlag_state;
//������־λ
uint8_t FFlag_state = 0;
extern u8 RFlag_state;
//�����˶�����
chassis_move_t chassis_move; //static
RampGen_t chassis_WRamp, chassis_ARamp, chassis_SRamp, chassis_DRamp;

static void chassis_init(chassis_move_t *chassis_move_init);//���̳�ʼ��
static void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode, remote_mode_e *inputmode);//����ģʽ����
static void chassis_feedback_update(chassis_move_t *chassis_move_update, remote_mode_e *inputmode); //�������ݸ���
static void chassis_Behaviour_update(chassis_move_t *chassis_behaviour_update);//������Ϊ����

#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t ChassisTaskStack;
#endif

//������
float output[4];
void chassis_task(void *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //���̳�ʼ��rgfg
    chassis_init(&chassis_move);
    RampInit(&chassis_WRamp, 0);
    RampInit(&chassis_ARamp, 0);
    RampInit(&chassis_SRamp, 0);
    RampInit(&chassis_DRamp, 0);

    //�жϵ��̵���Ƿ�����
//    while (toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE) || toe_is_error(DBUSTOE))
//    {
//        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
//    }
    while (1)
    {
        chassis_behaviour_mode_set(&chassis_move, &INPUTMOD);
        chassis_feedback_update(&chassis_move, &INPUTMOD);
        chassis_Behaviour_update(&chassis_move);
        Super_power_ctrl(&chassis_move);
        if (!(toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE)))
        {
            if(chassis_move.chassis_mode == CHASSIS_RELAX || toe_is_error(DBUSTOE))
            {
                CAN_CMD_CHASSIS(0, 0, 0, 0);
            }
            else
            {
//							 CAN_CMD_CHASSIS(0, 0, 0, 0);
							CAN_CMD_CHASSIS(chassis_move.motor_speed_pid[0].out, chassis_move.motor_speed_pid[1].out, chassis_move.motor_speed_pid[2].out, chassis_move.motor_speed_pid[3].out);
            }
        } 

        vTaskDelay(1);  //ϵͳ��ʱ
        #if INCLUDE_uxTaskGetStackHighWaterMark
        ChassisTaskStack = uxTaskGetStackHighWaterMark(NULL);
        #endif
				
			
    }
}
//���̳�ʼ��
fp32 follow_yaw_kp = 20.0f;
static void chassis_init(chassis_move_t *chassis_move_init)
{
    //�����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    //������ת��pidֵ
    const static fp32 chassis_follow_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    //�������ݻ�pidֵ
    const static fp32 dodge_pid[3] = {DODGE_PID_KP, DODGE_PID_KI, DODGE_PID_KD};

    if (chassis_move_init == NULL)
    {
        return;
    }

    uint8_t i;
    //���̿���״̬Ϊֹͣ
    chassis_move_init->chassis_mode = CHASSIS_INIT;
    //��ȡң����ָ��
    chassis_move_init->chassis_rc_ctrl = get_remote_control_point();
    //��������ָ̬��
    chassis_move_init->Gimbal_angle_gyro_point = get_Gyro_Angle_Point();
    //��ȡ��̨�������ָ��
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    //��ȡ����ϵͳ����ָ��
    chassis_move_init->chassis_status_measure = get_game_robot_state_t();
    chassis_move_init->chassis_power_measure = get_power_heat_data_t();
    chassis_move_init->chassis_hurt_type = get_robot_hurt_t();
    //��ȡ���ϵͳ����ָ��
    chassis_move_init->chassis_monitor_point = getErrorListPoint();

    //��ȡ����ͱ���������
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        chassis_move_init->motor_chassis[i].chassis_encoder_measure = get_Chassis_Encoder_Measure_Point(i);
        PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }

    //��ʼ����תPID
    PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_follow_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //����ģʽPID
    PID_Init(&chassis_move_init->dodge_pid, PID_POSITION, dodge_pid, CHASSIS_DODGE_PID_MAX_OUT, CHASSIS_DODGE_PID_MAX_IOUT);
}
//����ģʽ����
static void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode, remote_mode_e *inputmode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    if(*inputmode == REMOTE_INPUT)
    {
        if(switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //����
            chassis_move_mode->chassis_mode = CHASSIS_INIT; //��ʼ��
        }
        else if(switch_is_mid(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //����
            chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL; //���̸���
        }
        else if(switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //����
            chassis_move_mode->chassis_mode = CHASSIS_DODGE_MODE; //С����
        }
        else
        {
            chassis_move_mode->chassis_mode = CHASSIS_RELAX; //����
        }
    }
    /* �������ݸ���
    	F  ȡ��/��������>С����
    	shift	��סʹ��/�ɿ��رա���>��������
    	ctrl	��ס����
    */
    else if (*inputmode == KEYMOUSE_INPUT)
    {
        //++����ģʽ�� ��̨��ɳ�ʼ���־λ�����ֶ�����ģʽ
        if (!((chassis_move_mode->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F) != 0))
        {
            F_Flag = 1;
        }

        if (chassis_move_mode->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F && F_Flag == 1)
        {
            F_Flag = 0;
            FFlag_state ++;
            FFlag_state %= 2;
        }

        if(FFlag_state == 0 && GFlag_state == 0)
        {
//            if(ZFlag_state == 1)
//            {
//                chassis_move_mode->chassis_mode = CHASSIS_INIT;
//            }
//            else
//            {
                chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
//            }
        }

        if(GFlag_state == 1)
        {
            chassis_move_mode->chassis_mode = CHASSIS_SEPARATE_GIMBAL;
        }

        if(FFlag_state == 1 && GFlag_state == 0&&ZFlag_state==0&&RFlag_state==0)
        {
            chassis_move_mode->chassis_mode = CHASSIS_DODGE_MODE;
        }

        //�ɿ� Shift �رճ�������
        if ((!(chassis_move_mode->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT)) != 0)
        {
            Shift_Flag = 2;
        }

        //���� Shift ʹ�ó�������
        if ((chassis_move_mode->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT) && (Super_power.volt > 12000))
        {
            Shift_Flag = 1;
        }
        else
        {
            Shift_Flag = 2;
        }

        if ((chassis_move_mode->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL) != 0)
        {
            Ctrl_Flag = 1;
        }
        else
        {
            Ctrl_Flag = 0;
        }
    }
    else if (*inputmode == RUN_STOP)
    {
        chassis_move_mode->chassis_mode = CHASSIS_RELAX;
    }
}
//�������ݸ���
static void chassis_feedback_update(chassis_move_t *chassis_move_update, remote_mode_e *inputmode)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    //�ٶȷ���
    chassis_move_update->wheel_spd_fdb[0] = chassis_move_update->motor_chassis[0].chassis_motor_measure->speed_rpm;
    chassis_move_update->wheel_spd_fdb[1] = chassis_move_update->motor_chassis[1].chassis_motor_measure->speed_rpm;
    chassis_move_update->wheel_spd_fdb[2] = chassis_move_update->motor_chassis[2].chassis_motor_measure->speed_rpm;
    chassis_move_update->wheel_spd_fdb[3] = chassis_move_update->motor_chassis[3].chassis_motor_measure->speed_rpm;
//	chassis_move_update->wheel_spd_fdb[0]=chassis_move_update->motor_chassis[0].chassis_encoder_measure->filter_rate;
//	chassis_move_update->wheel_spd_fdb[1]=chassis_move_update->motor_chassis[1].chassis_encoder_measure->filter_rate;
//	chassis_move_update->wheel_spd_fdb[2]=chassis_move_update->motor_chassis[2].chassis_encoder_measure->filter_rate;
//	chassis_move_update->wheel_spd_fdb[3]=chassis_move_update->motor_chassis[3].chassis_encoder_measure->filter_rate;

    //ң�������ݸ���
    chassis_move_update->RC_X_ChassisSpeedRef = chassis_move_update->chassis_rc_ctrl->rc.ch[CHASSIS_X_CHANNEL] / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
    chassis_move_update->RC_Y_ChassisSpeedRef = chassis_move_update->chassis_rc_ctrl->rc.ch[CHASSIS_Y_CHANNEL] / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;
    chassis_move_update->RC_Z_ChassisSpeedRef = chassis_move_update->chassis_rc_ctrl->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_VZ_RC_SEN;

    if (*inputmode == KEYMOUSE_INPUT)
    {
        /* �������ݸ���
        	W S A D ǰ������
        */
        if (chassis_move_update->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_W)
        {
            W_Flag = 1;
        }

        if (!((chassis_move_update->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_W) != 0))
        {
            W_Flag = 0;
            RampResetCounter(&chassis_WRamp);
        }

        if (chassis_move_update->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_S)
        {
            S_Flag = 1;
        }

        if (!((chassis_move_update->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_S) != 0))
        {
            S_Flag = 0;
            RampResetCounter(&chassis_SRamp);
        }

        if (chassis_move_update->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_A)
        {
            A_Flag = 1;
        }

        if (!((chassis_move_update->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_A) != 0))
        {
            A_Flag = 0;
            RampResetCounter(&chassis_ARamp);
        }

        if (chassis_move_update->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_D)
        {
            D_Flag = 1;
        }

        if (!((chassis_move_update->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_D) != 0))
        {
            D_Flag = 0;
            RampResetCounter(&chassis_DRamp);
        }

        RampSetScale(&chassis_WRamp, 1000 );
        RampSetScale(&chassis_ARamp, 1000 );
        RampSetScale(&chassis_SRamp, 1000 );
        RampSetScale(&chassis_DRamp, 1000 );
        chassis_move_update->RC_Y_ChassisSpeedRef =  (W_Flag * RampCalc(&chassis_WRamp) * CHASSIS_KB_RC_MAX_SPEED + S_Flag * RampCalc(&chassis_SRamp) * (-CHASSIS_KB_RC_MAX_SPEED));
        chassis_move_update->RC_X_ChassisSpeedRef =  (A_Flag * RampCalc(&chassis_ARamp) * (-CHASSIS_KB_RC_MAX_SPEED) + D_Flag * RampCalc(&chassis_DRamp) * CHASSIS_KB_RC_MAX_SPEED);

//		chassis_move_update->RC_Y_ChassisSpeedRef = (W_Flag*CHASSIS_KB_RC_MAX_SPEED+S_Flag*(-CHASSIS_KB_RC_MAX_SPEED));
//		chassis_move_update->RC_X_ChassisSpeedRef = (A_Flag*(-CHASSIS_KB_RC_MAX_SPEED)+D_Flag*CHASSIS_KB_RC_MAX_SPEED);
        if(W_Flag == 1 && A_Flag == 1)
        {
            chassis_move_update->RC_X_ChassisSpeedRef = (-CHASSIS_KB_RC_MAX_SPEED) * 0.5f;
            chassis_move_update->RC_Y_ChassisSpeedRef = CHASSIS_KB_RC_MAX_SPEED * 0.5f;
        }
        else if(W_Flag == 1 && D_Flag == 1)
        {
            chassis_move_update->RC_X_ChassisSpeedRef = (CHASSIS_KB_RC_MAX_SPEED) * 0.5f;
            chassis_move_update->RC_Y_ChassisSpeedRef = CHASSIS_KB_RC_MAX_SPEED * 0.5f;
        }
        else if(S_Flag == 1 && A_Flag == 1)
        {
            chassis_move_update->RC_X_ChassisSpeedRef = (-CHASSIS_KB_RC_MAX_SPEED) * 0.5f;
            chassis_move_update->RC_Y_ChassisSpeedRef = -CHASSIS_KB_RC_MAX_SPEED * 0.5f;
        }
        else if(S_Flag == 1 && D_Flag == 1)
        {
            chassis_move_update->RC_X_ChassisSpeedRef = (CHASSIS_KB_RC_MAX_SPEED) * 0.5f;
            chassis_move_update->RC_Y_ChassisSpeedRef = -CHASSIS_KB_RC_MAX_SPEED * 0.5f;
        }
    }
}
//����״̬����
fp32 vz_t = 0, Rotation_rate = 0.0f;
int32_t vz_count = 0;
static void chassis_Behaviour_update(chassis_move_t *chassis_behaviour_update)
{
    uint8_t i = 0;
    fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

    if (chassis_behaviour_update == NULL)
    {
        return;
    }

    if(chassis_behaviour_update->chassis_mode == CHASSIS_RELAX)
    {
        for(i = 0; i < 4; i++)
        {
            chassis_move.motor_speed_pid[i].out = 0;
        }
    }
    else if(chassis_behaviour_update->chassis_mode == CHASSIS_INIT)
    {
        chassis_behaviour_update->vx = 0;
        chassis_behaviour_update->vy = 0;
        chassis_behaviour_update->vz = 0;
        mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
    }
    //���̸���
    else if(chassis_behaviour_update->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
    {
        float FOLLOW_GIMBAL_Z;//������ת
//                                                                          ��ԽǶ�
        sin_yaw = arm_sin_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);
        cos_yaw = arm_cos_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);

        chassis_behaviour_update->vx = (cos_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + sin_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef);
        chassis_behaviour_update->vy = (-sin_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + cos_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef);
        chassis_behaviour_update->chassis_relative_angle_set = 0.0f;
//		RAMP_float(0.0f,chassis_behaviour_update->chassis_yaw_motor->relative_angle,13.0f);//б�¼��ٸ�����Ӧ
        FOLLOW_GIMBAL_Z = PID_Calc(&chassis_behaviour_update->chassis_angle_pid, chassis_behaviour_update->chassis_yaw_motor->relative_angle, chassis_behaviour_update->chassis_relative_angle_set);
//		RAMP_float(chassis_behaviour_update->chassis_relative_angle_set, chassis_behaviour_update->chassis_yaw_motor->relative_angle, 5.5f),//б�¼��ٸ��涶��
        /* Ťͷ�ٶ�Խ��,ǰ���ٶ�Խ��,��ֹת��뾶���� */
        {
            if( fabs(FOLLOW_GIMBAL_Z) > 160.0f)//210
            {
                Rotation_rate = ((MAX_CHASSIS_VR_SPEED - fabs(FOLLOW_GIMBAL_Z) - 4500.0f) / MAX_CHASSIS_VR_SPEED) * ((MAX_CHASSIS_VR_SPEED - fabs(FOLLOW_GIMBAL_Z) - 4500.0f) / MAX_CHASSIS_VR_SPEED);
            }//5840
            else
            {
                Rotation_rate = 1.0f;
            }

//		VAL_LIMIT(Rotation_rate,0,1);
            chassis_behaviour_update->vx = Rotation_rate * fp32_constrain(chassis_behaviour_update->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
            chassis_behaviour_update->vy = Rotation_rate * fp32_constrain(chassis_behaviour_update->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
            chassis_behaviour_update->vz = fp32_constrain(FOLLOW_GIMBAL_Z, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s
        }
        mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
    }
    //С����
    else if(chassis_behaviour_update->chassis_mode == CHASSIS_DODGE_MODE)
    {
        sin_yaw = arm_sin_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);
        cos_yaw = arm_cos_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);

        chassis_behaviour_update->vx = (cos_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + sin_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef) * 0.27f;
        chassis_behaviour_update->vy = (-sin_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + cos_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef) * 0.27f;

//		vz_count++;
//		if(vz_count%50== 0)
//		{
//			vz_t++;
//		}
        chassis_behaviour_update->vz = 350.0f;
//		chassis_behaviour_update->vz=350.0f * (arm_sin_f32( 7.0f * vz_t * ANGLE_TO_RAD)) + 500.0f;
        mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
    }
    //���̷���
    else if(chassis_behaviour_update->chassis_mode == CHASSIS_SEPARATE_GIMBAL)
    {
        chassis_behaviour_update->vx = chassis_behaviour_update->RC_X_ChassisSpeedRef;
        chassis_behaviour_update->vy = chassis_behaviour_update->RC_Y_ChassisSpeedRef;
        chassis_behaviour_update->vz = chassis_behaviour_update->RC_Z_ChassisSpeedRef;

        mecanum_calc(chassis_behaviour_update->vx, chassis_behaviour_update->vy, chassis_behaviour_update->vz, chassis_behaviour_update->wheel_spd_ref, chassis_behaviour_update);
    }

    for (u8 i = 0; i < 4; i++)
    {
      PID_Calc(&chassis_behaviour_update->motor_speed_pid[i], chassis_behaviour_update->wheel_spd_fdb[i], chassis_behaviour_update->wheel_spd_ref[i]);
    }

    Chassis_Power_Limit(chassis_behaviour_update);
}

