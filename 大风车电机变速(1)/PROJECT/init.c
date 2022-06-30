#include "init.h"
#include "delay.h"
#include "string.h"
/*˵��
 *����������Ϊ3���ӱ���
 *TIM3��ʱ
 *Big_Pinwheel ��С�������ر�־λ 1Ϊ���������� 0ΪС��������
*/
#define  Big_Pinwheel  1
extern motor_measure_t motormeaser;
extern Encoder_process_t motorfif;
PID pid1;
void NVIC_StateInit(void );
void ThreadInit(void);
extern CanRxMsg  Rx_message;
static float p=5,i=0,d=0;
extern int t;
//void delay_ms(u32 time);
void KernelTaskInit(void)
{
	/************************************************/
	//�����ʼ������
	/************************************************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	CANTransfer_Init();
	TIM3_PWM_Init((10000-1),7199);
	Q_OFF;
	NVIC_StateInit();
	delay_init();
	for(;;)
	{	
		static char count = 0;
		PID_Set(&pid1,p,i,d,15000);
#if (Big_Pinwheel==1)
		if(count >= 1)
		{
			pid1.ref = -(float)((((0.785*sin(1.884*t)+1.305)/0.1047)*27.0f*72.0f/21.0f)/0.9);
			if(count == 2)
				count = 0;
		}
		else
		{
		pid1.ref=(float)((((0.785*sin(1.884*t)+1.305)/0.1047)*27.0f*72.0f/21.0f)/0.9);
		}
		//�������RPM ָ �������ٶ�RPM 
//				pid1.ref=0.0f;
#elif (Big_Pinwheel==0)
		pid1.ref = (10*27.0f*72.0f/21.0f)*1.1;
#endif
		pid1.fdb = motormeaser.speed_rpm;
		PID_Control(&pid1); 
		CAN_send_2006(pid1.pid_out);
		delay_ms(5);
		if(t==180)
		 {
			t=0;
			count++;
		 }
	}
}


void NVIC_StateInit(void )
{
	NVIC_InitTypeDef 		Nvic_InitStructure;
	/* Configure one bit for preemption priority */
	/*�ж�����*/
	
	Nvic_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	Nvic_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	Nvic_InitStructure.NVIC_IRQChannelSubPriority=0;
	Nvic_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&Nvic_InitStructure);
	
	Nvic_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   //CAN1 RX0�ж�
	Nvic_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;		   //��ռ���ȼ�0
	Nvic_InitStructure.NVIC_IRQChannelSubPriority = 0;			  		 //�����ȼ�Ϊ0
	Nvic_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&Nvic_InitStructure);
//	
//	Nvic_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;	  	 //CAN1 TX�ж�
//	Nvic_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;		   //��ռ���ȼ�0
//	Nvic_InitStructure.NVIC_IRQChannelSubPriority = 1;					   //�����ȼ�Ϊ0
//	Nvic_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&Nvic_InitStructure);
	
}
//extern float ONE_L;

extern PID PID_R;
//extern int Motor_Pwm;
//extern int Motor_pwm;




//float rc_to_angle(Remote rc)
//{
//	float angle;
//	angle = (float)((rc.ch0 - 364)*360/1320);
//	return angle;
//}
//	
//float adc_to_angle(u32 adc)
//{
//	float angle;
//	angle = adc*360/4096;
//	return angle;
//}


	
	
	
	
	
