#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "stm32f10x.h"

extern uint8_t TP_enable_irq_flag;

#define TP_IRQ_ENABLE()		{TP_enable_irq_flag=1;EXTI_ClearITPendingBit(EXTI_Line1);NVIC_EnableIRQ(EXTI1_IRQn);}
#define TP_IRQ_DISABLE()	{TP_enable_irq_flag=0;NVIC_DisableIRQ(EXTI1_IRQn);}

#define TP_PRES_DOWN 0x80  		//����������	  
#define TP_CATH_PRES 0x40  		//�а�������

//������������
typedef struct 
{
	u8 (*init)(void);			//��ʼ��������������
	u8 (*scan)(u8);				//ɨ�败����.0,��Ļɨ��;1,��������;	 
	void (*adjust)(void);	//������У׼ 
	u16 x; 		//��ǰ����
	u16 y;		//�����������5������,����������x[0],y[0]����:�˴�ɨ��ʱ,����������,��
						//x[4],y[4]�洢��һ�ΰ���ʱ������. 
	u8  sta;	//�ʵ�״̬ 
						//b7:����1/�ɿ�0; 
	          //b6:0,û�а�������;1,�а�������. 
						//b5:����
						//b4~b0:���ݴ��������µĵ���(0,��ʾδ����,1��ʾ����)
/////////////////////������У׼����(����������ҪУ׼)//////////////////////								
	float xfac;					
	float yfac;
	short xoff;
	short yoff;	   
//�����Ĳ���,��������������������ȫ�ߵ�ʱ��Ҫ�õ�.
//b0:0,����(�ʺ�����ΪX����,����ΪY�����TP)
//   1,����(�ʺ�����ΪY����,����ΪX�����TP) 
//b1~6:����.
//b7:0,������
//   1,������ 
	u8 touchtype;
	u8 adjust_flag;
}_m_tp_dev;

extern _m_tp_dev tp_dev;	 	//������������touch.c���涨��


void TP_Adjust(void);
uint8_t TP_Init(void);
uint8_t TP_Scan(uint8_t tp);
uint8_t TP_Area_Press_Judge(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey);


#endif
