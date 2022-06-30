#include "chassis_task.h"

//任务开始空闲一段时间
#define 	CHASSIS_TASK_INIT_TIME   100
//遥控输入模式设定
extern remote_mode_e INPUTMOD;
extern u8 GFlag_state;
extern u8 ZFlag_state;
//按键标志位
uint8_t FFlag_state = 0;
extern u8 RFlag_state;
//底盘运动数据
chassis_move_t chassis_move; //static
RampGen_t chassis_WRamp, chassis_ARamp, chassis_SRamp, chassis_DRamp;

static void chassis_init(chassis_move_t *chassis_move_init);//底盘初始化
static void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode, remote_mode_e *inputmode);//底盘模式设置
static void chassis_feedback_update(chassis_move_t *chassis_move_update, remote_mode_e *inputmode); //底盘数据更新
static void chassis_Behaviour_update(chassis_move_t *chassis_behaviour_update);//底盘行为更新

#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t ChassisTaskStack;
#endif

//主任务
float output[4];
void chassis_task(void *pvParameters)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘初始化rgfg
    chassis_init(&chassis_move);
    RampInit(&chassis_WRamp, 0);
    RampInit(&chassis_ARamp, 0);
    RampInit(&chassis_SRamp, 0);
    RampInit(&chassis_DRamp, 0);

    //判断底盘电机是否都在线
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

        vTaskDelay(1);  //系统延时
        #if INCLUDE_uxTaskGetStackHighWaterMark
        ChassisTaskStack = uxTaskGetStackHighWaterMark(NULL);
        #endif
				
			
    }
}
//底盘初始化
fp32 follow_yaw_kp = 20.0f;
static void chassis_init(chassis_move_t *chassis_move_init)
{
    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    //底盘旋转环pid值
    const static fp32 chassis_follow_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    //底盘陀螺环pid值
    const static fp32 dodge_pid[3] = {DODGE_PID_KP, DODGE_PID_KI, DODGE_PID_KD};

    if (chassis_move_init == NULL)
    {
        return;
    }

    uint8_t i;
    //底盘开机状态为停止
    chassis_move_init->chassis_mode = CHASSIS_INIT;
    //获取遥控器指针
    chassis_move_init->chassis_rc_ctrl = get_remote_control_point();
    //陀螺仪姿态指针
    chassis_move_init->Gimbal_angle_gyro_point = get_Gyro_Angle_Point();
    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    //获取裁判系统数据指针
    chassis_move_init->chassis_status_measure = get_game_robot_state_t();
    chassis_move_init->chassis_power_measure = get_power_heat_data_t();
    chassis_move_init->chassis_hurt_type = get_robot_hurt_t();
    //获取检测系统数据指针
    chassis_move_init->chassis_monitor_point = getErrorListPoint();

    //获取电机和编码器数据
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        chassis_move_init->motor_chassis[i].chassis_encoder_measure = get_Chassis_Encoder_Measure_Point(i);
        PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }

    //初始化旋转PID
    PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_follow_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //闪避模式PID
    PID_Init(&chassis_move_init->dodge_pid, PID_POSITION, dodge_pid, CHASSIS_DODGE_PID_MAX_OUT, CHASSIS_DODGE_PID_MAX_IOUT);
}
//底盘模式设置
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
            //左下
            chassis_move_mode->chassis_mode = CHASSIS_INIT; //初始化
        }
        else if(switch_is_mid(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //左中
            chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL; //底盘跟随
        }
        else if(switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[ModeChannel_L]))
        {
            //左上
            chassis_move_mode->chassis_mode = CHASSIS_DODGE_MODE; //小陀螺
        }
        else
        {
            chassis_move_mode->chassis_mode = CHASSIS_RELAX; //无力
        }
    }
    /* 键盘数据更新
    	F  取消/启动——>小陀螺
    	shift	按住使用/松开关闭——>超级电容
    	ctrl	按住减速
    */
    else if (*inputmode == KEYMOUSE_INPUT)
    {
        //++键鼠模式下 云台完成初始后标志位进入手动控制模式
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

        //松开 Shift 关闭超级电容
        if ((!(chassis_move_mode->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT)) != 0)
        {
            Shift_Flag = 2;
        }

        //按下 Shift 使用超级电容
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
//底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update, remote_mode_e *inputmode)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    //速度反馈
    chassis_move_update->wheel_spd_fdb[0] = chassis_move_update->motor_chassis[0].chassis_motor_measure->speed_rpm;
    chassis_move_update->wheel_spd_fdb[1] = chassis_move_update->motor_chassis[1].chassis_motor_measure->speed_rpm;
    chassis_move_update->wheel_spd_fdb[2] = chassis_move_update->motor_chassis[2].chassis_motor_measure->speed_rpm;
    chassis_move_update->wheel_spd_fdb[3] = chassis_move_update->motor_chassis[3].chassis_motor_measure->speed_rpm;
//	chassis_move_update->wheel_spd_fdb[0]=chassis_move_update->motor_chassis[0].chassis_encoder_measure->filter_rate;
//	chassis_move_update->wheel_spd_fdb[1]=chassis_move_update->motor_chassis[1].chassis_encoder_measure->filter_rate;
//	chassis_move_update->wheel_spd_fdb[2]=chassis_move_update->motor_chassis[2].chassis_encoder_measure->filter_rate;
//	chassis_move_update->wheel_spd_fdb[3]=chassis_move_update->motor_chassis[3].chassis_encoder_measure->filter_rate;

    //遥控器数据更新
    chassis_move_update->RC_X_ChassisSpeedRef = chassis_move_update->chassis_rc_ctrl->rc.ch[CHASSIS_X_CHANNEL] / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
    chassis_move_update->RC_Y_ChassisSpeedRef = chassis_move_update->chassis_rc_ctrl->rc.ch[CHASSIS_Y_CHANNEL] / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;
    chassis_move_update->RC_Z_ChassisSpeedRef = chassis_move_update->chassis_rc_ctrl->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_VZ_RC_SEN;

    if (*inputmode == KEYMOUSE_INPUT)
    {
        /* 键盘数据更新
        	W S A D 前后左右
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
//底盘状态更新
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
    //底盘跟随
    else if(chassis_behaviour_update->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
    {
        float FOLLOW_GIMBAL_Z;//左右旋转
//                                                                          相对角度
        sin_yaw = arm_sin_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);
        cos_yaw = arm_cos_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle * ANGLE_TO_RAD);

        chassis_behaviour_update->vx = (cos_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + sin_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef);
        chassis_behaviour_update->vy = (-sin_yaw * chassis_behaviour_update->RC_X_ChassisSpeedRef + cos_yaw * chassis_behaviour_update->RC_Y_ChassisSpeedRef);
        chassis_behaviour_update->chassis_relative_angle_set = 0.0f;
//		RAMP_float(0.0f,chassis_behaviour_update->chassis_yaw_motor->relative_angle,13.0f);//斜坡减少跟随响应
        FOLLOW_GIMBAL_Z = PID_Calc(&chassis_behaviour_update->chassis_angle_pid, chassis_behaviour_update->chassis_yaw_motor->relative_angle, chassis_behaviour_update->chassis_relative_angle_set);
//		RAMP_float(chassis_behaviour_update->chassis_relative_angle_set, chassis_behaviour_update->chassis_yaw_motor->relative_angle, 5.5f),//斜坡减少跟随抖动
        /* 扭头速度越快,前后速度越慢,防止转弯半径过大 */
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
    //小陀螺
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
    //底盘分离
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

