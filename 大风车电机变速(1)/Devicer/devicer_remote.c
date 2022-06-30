#include "devicer_remote.h"
#include "delay.h"
volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0};
RC_Ctl_t RC_CtrlData;   	//remote control data

PID PID_R;
PID PID_S;
extern int Motor_Pwm;
extern int Motor_pwm;
/****
下面命名方式
自下而上分别将所有臂分为1-6
L代表左
R代表右
I代表小拨轮
*****/

float ONE_L = 0;
float TWO_L = 0;
float THREE_L = 0;
float FOUR_R = 1500.0f;
float FIVE_R = 1500.0f;

float SIX_I = 0;



u8 current_mode=0;
u8 last_mode=0;

u8 move_cmd=0;

void CAN_recive_2006(CanRxMsg *msg)				//调试版中，Can那个界面里头的电机反馈
{
    switch(msg->StdId)
    {
    case 0x201:
        EncoderProcess(&CM1Encoder ,msg);       //获取到编码器的初始偏差值
        break;
    case 0x202:
        //在这个地方写上遥控器数值接收的程序
        break;
    case 0x203:

        break;
    case 0x204:

        break;
    default:
        break;
    }
}
void int_ceshi(void);

//UP_1  MID_3  DOWN_2
void SetInputMode(Remote *rc)
{
    if(rc->s1==2&&rc->s2 == 2)      //双下
    {
//		init_set();
        Reset_stop;

    }
    else if(rc->s1==1&&rc->s2 == 3)//左上右中
    {
//		LIFT_THREE();
        Reset_SET;
    }
    else if(rc->s1==2&&rc->s2 == 3)//左下右中
    {
//        int_ceshi();
        init_run();
        Reset_SET;
//		init_run();
    }
    else if(rc->s1==3&&rc->s2 == 2)//左中右下
    {
        Reset_SET;
    }
    else if(rc->s1==3&&rc->s2 == 3)//左中右中
    {
        init_run();
        Reset_SET;
//		LIFT_ONE();
    }
    else if(rc->s1==3&&rc->s2 == 1)//左中右上
    {
        Reset_SET;
    }
    else if(rc->s1==1&&rc->s2 == 1)//左上右上
    {
        
        show_mode();
        Reset_SET;
    }

}

void Selectmode(uint8_t ie)
{
}

void show_mode(void)
{
    if(((RC_CtrlData.rc.ch4-1024)<200)&&(RC_CtrlData.rc.ch4-1024)>-200)
    {
        current_mode=0;
    }
  
    else if((RC_CtrlData.rc.ch4-1024)<=-200)
    {
        current_mode=1;
    }
    else if((RC_CtrlData.rc.ch4-1024)>=200)
    {
        current_mode=2;
    }
    
    if ((last_mode != 0) && current_mode == 0)
    {
            
    }

    else if ((last_mode != 1) && current_mode == 1)
    {
        move_cmd=1;
    }
    else if ((last_mode != 2) && current_mode == 2)
    {
        move_cmd=2;
    }
    last_mode = current_mode;

    if(move_cmd==1)
    {
        /*动动中臂*/
        Motor_pwm=13;
        Change_Rotatepwm();
        delay_ms(1000);
				delay_ms(1000);
				
        
        Motor_pwm=0;
        Change_Rotatepwm();

        /*动动大臂*/
        Motor_Pwm=-5;
        Change_RotatePWM();
        delay_ms(1000);
				delay_ms(1000);
//				delay_ms(1000);
        
        Motor_Pwm=0;
        Change_RotatePWM();
        
        /*动动小臂*/
    //    TIM_SetCompare2(TIM2,SERVO_MAX);
    //    delay_ms(500);
    //    TIM_SetCompare2(TIM2,SERVO_MIN);
    //    delay_ms(500);
        
        /*动动手腕*/
        TIM_SetCompare1(TIM2,SERVO_MAX);
        delay_ms(500);
        TIM_SetCompare1(TIM2,SERVO_MIN);
        delay_ms(500);
        
        /*动动夹手*/
        Q_ON;
        delay_ms(500);
        Q_OFF;
        delay_ms(500);
        
        
        move_cmd=0;
    }
    
//    if(move_cmd==2)   /*回收*/
//    {
//        
////            Motor_pwm=13;
////            Change_Rotatepwm();
//        
////        delay_ms(3000);
//        
//        Motor_pwm=0;
//        Change_Rotatepwm();
//        while(S_1==1)
//        {
//            /*动动大臂*/
//            Motor_Pwm=4;
//            Change_RotatePWM();
//            delay_ms(2000);
//            
//            
//        }
//        Motor_Pwm=0;
//        Change_RotatePWM();
//    
//        
//        TIM_SetCompare1(TIM2,SERVO_INIT);
//        delay_ms(500);
//       
//        Q_OFF;
//        
//        
//        move_cmd=0;
//    }
}

void int_ceshi(void)   //左下右中
{
//    float add_pit = 0.0f;
    
    ONE_L = (RC_CtrlData.rc.ch0-1024)*6.5f;				/*底盘*/	//左 左右      //右横轴  -660~660   -4290~4290
//    FOUR_R = (RC_CtrlData.rc.ch1-364)/12;				/**/	//左 上下      //右竖轴  0~1320   0~110
    Motor_Pwm = (RC_CtrlData.rc.ch2-1024)*0.02f;        /*大臂*/                   //左横轴  -660~660   0~13.2   
    Motor_pwm = (RC_CtrlData.rc.ch3-1024)*0.02f;        /*中臂*/                    //左竖轴  -660~660   0~13.2
    SIX_I = (RC_CtrlData.rc.ch4-1024)*0.5f;					//拨轮，上夹，下放

//    add_pit=(RC_CtrlData.rc.ch1-1024)*0.005f;
//    FOUR_R+=add_pit;
//    FOUR_R=LIMIT(FOUR_R,SERVO_MIN,SERVO_MAX);
    
//    TIM_SetCompare2(TIM2,FOUR_R);

    qigang(SIX_I);
//	init_PWM_Set();
//	M2006_set(ONE_L,5);
}


//void init_PWM_Set(void)
//{
//    if((C_1==0)||(C_2==0))
//    {
//        Motor_Pwm = 0;
//    }
//    else {

//    }

//    if((C_3==0)||(C_4==0))
//    {
//        Motor_pwm = 0;
//    }
//    else
//    {

//    }
//}
void init_run(void)   //左中右中
{
    float add_roll = 0.0f;
    
    ONE_L = (RC_CtrlData.rc.ch0-1024)*6.5f;				/*底盘*/	//左 左右      //右横轴  -660~660   -4290~4290

//    float add_pitch = 0.0f;
//    FIVE_R = absolute_value((RC_CtrlData.rc.ch0-364)/12);	//左 舵机   //右横轴  0~1320   0~110      //??
//    FOUR_R = absolute_value((RC_CtrlData.rc.ch1-364)/12);	//左 舵机轴 //右竖轴  0~1320   0~110
    
    Motor_Pwm = (RC_CtrlData.rc.ch2-1024)*0.02f;                        //左横轴  -660~660   0~13.2   //哪一个电机
    Motor_pwm = (RC_CtrlData.rc.ch3-1024)*0.02f;                        //左竖轴  -660~660   0~13.2
    SIX_I = (RC_CtrlData.rc.ch4-1024)*0.5f;					//拨轮，上夹，下放

    
    add_roll=(RC_CtrlData.rc.ch1-1024)*0.005f;
//    add_pitch=(RC_CtrlData.rc.ch1-1024)*0.005f;
//    move_control(&add_yaw,&add_pitch,gimbal_set);
    FIVE_R+=add_roll;
//    FOUR_R+=add_pitch;
    
    FIVE_R=LIMIT(FIVE_R,SERVO_MIN,SERVO_MAX);
//    FOUR_R=LIMIT(FOUR_R,SERVO_MIN,SERVO_MAX);

    
    TIM_SetCompare1(TIM2,FIVE_R);                            //??
//    TIM_SetCompare2(TIM2,FOUR_R);
    qigang(SIX_I);

}

void LIFT_THREE()
{
}

void LIFT_ONE()
{
}



void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
    v->last_raw_value = v->raw_value;
    v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
    v->speeed_rpm=(msg->Data[2]<<8)|msg->Data[3];
    v->current=(msg->Data[4]<<8)|msg->Data[5];
    v->diff = v->raw_value - v->last_raw_value;
    if(v->first)
    {
        if(v->diff < -6500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
        {
            v->round_cnt++;
        }
        else if(v->diff>6500)
        {
            v->round_cnt--;
        }
        else
        {
        }
    }
    else
    {
        v->first++;
    }
}

void M2006_set(float FM1,u8 mod)
{
    PID_R.ref = FM1;
    PID_R.fdb = CM1Encoder.speeed_rpm;
    PID_Set(&PID_R,3,0.005f,0,2000);
    PID_Control(&PID_R);
    VAL_LIMIT(PID_R.pid_out,-30000,30000);
//	if(mod == 1) 														//操控模式
//	{
//		if((FM1>0)&&(C_1!=1)&&(C_4!=1))
//		{
//			CAN_send_2006((int16_t)PID_R.pid_out);
//		}
//		else if((FM1<0)&&(C_1!=1)&&(C_4!=1))
//		{
//			CAN_send_2006((int16_t)PID_R.pid_out);
//		}
//		else
//		{
//			PID_S.ref = 0;
//			PID_S.fdb = CM1Encoder.speeed_rpm;
//			PID_Set(&PID_S,3,0.5f,0,2000);
//			PID_Control(&PID_S);
//			VAL_LIMIT(PID_S.pid_out,-3000,3000);
//			CAN_send_2006(PID_S.pid_out);
//		}
//	}
//	else if(mod == 0)
//	{
//		if((FM1>0)&&(C_4!=1))
//		{
//			CAN_send_2006((int16_t)PID_R.pid_out);
//		}
//		else if((FM1<0)&&(C_1!=1))
//		{
//			CAN_send_2006((int16_t)PID_R.pid_out);
//		}
//		else
//		{
//			PID_S.ref = 0;
//			PID_S.fdb = CM1Encoder.speeed_rpm;
//			PID_Set(&PID_S,3,0.5f,0,2000);
//			PID_Control(&PID_S);
//			VAL_LIMIT(PID_S.pid_out,-3000,3000);
//			CAN_send_2006(PID_S.pid_out);
//		}
//	}
//	else{
    CAN_send_2006((int16_t)PID_R.pid_out);
//	}
}

void zhiliu(float FM1)
{

}

void diangang(float FM1)
{
}

void qigang(float FA)
{
    if(FA>200)
    {
        Q_ON;
    }
    else if(FA<-200)
    {
        Q_OFF;
    }
}



void RemoteDataPrcess(uint8_t *pData)
{

    if(pData == 0)
    {
        return;
    }
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 						//364~1024~1684
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;			//364~1024~1684
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |								//364~1024~1684
                          ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;				//364~1024~1684
    /*********s1，s2的1、2、3分别对应上、下、中************************************************************/
    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;    //左
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);         //右

    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);

    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];

    RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
    RC_CtrlData.rc.ch4 = ((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF;
//    SetInputMode(&RC_CtrlData.rc);
}

