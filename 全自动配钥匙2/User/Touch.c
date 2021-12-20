#include "Touch.h"
#include "LCD.h"
#include "math.h"
#include "EEPROM.h" 
#include "mydelay.h"


#define TP_IRQ_Read()		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)
//#define TP_MISO_Read()	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)
//#define TP_MOSI_H()			GPIO_SetBits(GPIOE,GPIO_Pin_4)
//#define TP_MOSI_L()			GPIO_ResetBits(GPIOE,GPIO_Pin_4)
//#define TP_SCK_H()			GPIO_SetBits(GPIOE,GPIO_Pin_5)
//#define TP_SCK_L()			GPIO_ResetBits(GPIOE,GPIO_Pin_5)
#define TP_CS_H()				GPIO_SetBits(GPIOB,GPIO_Pin_12)
#define TP_CS_L() 			GPIO_ResetBits(GPIOB,GPIO_Pin_12)

 

_m_tp_dev tp_dev=
{
	TP_Init,
	TP_Scan,
	TP_Adjust,
	0,
	0, 
	0,
	0,
	0,
	0,	  	 		
	0,
	0,	  	 		
};					
//Ĭ��Ϊtouchtype=0������.
uint8_t TP_CMD_RDX=0X90;
uint8_t TP_CMD_RDY=0XD0;

uint8_t TP_enable_irq_flag=0;


uint8_t TP_SPI_RW_Byte(uint8_t byte)
{
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(SPI2,byte);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
	return SPI_I2S_ReceiveData(SPI2);
}
/*
//SPIд����
//������ICд��1byte����    
//num:Ҫд�������
void TP_Write_Byte(uint8_t num)    
{  
	uint8_t count=0;   
	for(count=0;count<8;count++)  
	{ 	  
		if(num&0x80)
			TP_MOSI_H();  
		else 
			TP_MOSI_L();   
		num<<=1;    
		TP_SCK_L(); 
		my_delayus(1);
		TP_SCK_H();		//��������Ч	        
	}		 			    
}*/

//SPI������ 
//�Ӵ�����IC��ȡadcֵ
//CMD:ָ��
//����ֵ:����������	   
u16 TP_Read_AD(uint8_t CMD)	  
{ 	   
	u16 Num=0; 
	
	TP_CS_L();  	//ѡ�д�����IC
	TP_SPI_RW_Byte(CMD);//����������
	my_delayus(6);//ADS7846��ת��ʱ���Ϊ6us 	    
	Num=TP_SPI_RW_Byte(0x00);
	Num=Num<<8;	
	Num|=TP_SPI_RW_Byte(0x00);
	Num>>=3;   	
	TP_CS_H();		//�ͷ�Ƭѡ	 
	return(Num);   
}

//��ȡһ������ֵ(x����y)
//������ȡREAD_TIMES������,����Щ������������,
//Ȼ��ȥ����ͺ����LOST_VAL����,ȡƽ��ֵ 
//xy:ָ�CMD_RDX/CMD_RDY��
//����ֵ:����������
#define READ_TIMES 5 	//��ȡ����
#define LOST_VAL 1	  	//����ֵ
u16 TP_Read_XOY(uint8_t xy)
{
	u16 i, j;
	u16 buf[READ_TIMES];
	u16 sum=0;
	u16 temp;
	for(i=0;i<READ_TIMES;i++)
		buf[i]=TP_Read_AD(xy);		 		    
	for(i=0;i<READ_TIMES-1; i++)//����
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//��������
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)
		sum+=buf[i];
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
}

//��ȡx,y����
//x,y:��ȡ��������ֵ
//����ֵ:0,ʧ��;1,�ɹ���
uint8_t TP_Read_XY(u16 *x,u16 *y)
{
	u16 xtemp,ytemp;			 	 		  
	xtemp=TP_Read_XOY(TP_CMD_RDX);
	ytemp=TP_Read_XOY(TP_CMD_RDY);	  												   
	
	*x=xtemp;
	*y=ytemp;
	return 1;//�����ɹ�
}

//����2�ζ�ȡ������IC,�������ε�ƫ��ܳ���
//ERR_RANGE,��������,����Ϊ������ȷ,�����������.	   
//�ú����ܴ�����׼ȷ��
//x,y:��ȡ��������ֵ
//����ֵ:0,ʧ��;1,�ɹ���
#define ERR_RANGE 50 //��Χ 
uint8_t TP_Read_XY2(u16 *x,u16 *y) 
{
	u16 x1,y1;
 	u16 x2,y2;
 	uint8_t flag;    
	flag=TP_Read_XY(&x1,&y1);   
	if(flag==0)
		return(0);
	flag=TP_Read_XY(&x2,&y2);	   
	if(flag==0)
		return(0);   
	if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//ǰ�����β�����+-50��
	&&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
	{
			*x=(x1+x2)/2;
			*y=(y1+y2)/2;
			return 1;
	}
	else 
		return 0;	  
} 

//��������ɨ��
//tp:0,��Ļ����;1,��������(У׼�����ⳡ����)
//����ֵ:��ǰ����״̬.
//0,�����޴���;1,�����д���
uint8_t TP_Scan(uint8_t tp)
{			 

	if(TP_IRQ_Read()==0)//  ----------------------------																																																																															b�а�������
	{
		if(tp)
			TP_Read_XY2(&tp_dev.x,&tp_dev.y);//��ȡ��������
		else if(TP_Read_XY2(&tp_dev.x,&tp_dev.y))//��ȡ��Ļ����
		{
//	 		tp_dev.x=tp_dev.x/4.50f-60;//�����ת��Ϊ��Ļ����     tp_dev.xfac*   tp_dev.yfac*
//			tp_dev.y=tp_dev.y/7.80f-40;  
			tp_dev.x=tp_dev.xfac*tp_dev.x+tp_dev.xoff;//�����ת��Ϊ��Ļ����
			tp_dev.y=tp_dev.yfac*tp_dev.y+tp_dev.yoff;  
	 	} 
		if((tp_dev.sta&TP_PRES_DOWN)==0)//֮ǰû�б�����
		{		 
			tp_dev.sta=TP_PRES_DOWN|TP_CATH_PRES;//��������    	   			 
		}			   
	}
	else
	{
		if(tp_dev.sta&TP_PRES_DOWN)//֮ǰ�Ǳ����µ�
		{
			tp_dev.sta&=~TP_PRES_DOWN;//��ǰ����ɿ�	
		}
		else//֮ǰ��û�б�����
		{
			tp_dev.x=0xffff;
			tp_dev.y=0xffff;
		}	    
	}
	return tp_dev.sta&TP_PRES_DOWN;//���ص�ǰ�Ĵ���״̬
}	 

//����У׼����										    
void TP_Save_Adjdata(void)
{
	tp_dev.adjust_flag=0xAA;
	AT24CXX_Write(0,(uint8_t*)&tp_dev.xfac,14);	//ǿ�Ʊ���&tp_dev.xfac��ַ��ʼ��14���ֽ�����
}

//�õ�������EEPROM�����У׼ֵ
//����ֵ��1���ɹ���ȡ����
//        0����ȡʧ�ܣ�Ҫ����У׼
uint8_t TP_Get_Adjdata(void)
{					  
	AT24CXX_Read(0,(uint8_t*)&tp_dev.xfac,14);//��ȡ֮ǰ�����У׼���� 
	if(tp_dev.adjust_flag==0xAA)//�������Ѿ�У׼����			   
 	{ 
		if(tp_dev.touchtype)//X,Y��������Ļ�෴
		{
			TP_CMD_RDX=0X90;
			TP_CMD_RDY=0XD0;	 
		}
		else				   //X,Y��������Ļ��ͬ
		{
			TP_CMD_RDX=0XD0;
			TP_CMD_RDY=0X90;	 
		}		 
		return 1;	 
	}
	return 0;
}



//��һ��������
//����У׼�õ�
//x,y:����
//color:��ɫ
void TP_Drow_Touch_Point(u16 x,u16 y,u16 color)
{
	LCD_brush_color=color;
	LCD_DrawLine(x-12,y,x+13,y);//����
	LCD_DrawLine(x,y-12,x,y+13);//����
	LCD_DrawPoint(x+1,y+1);
	LCD_DrawPoint(x-1,y+1);
	LCD_DrawPoint(x+1,y-1);
	LCD_DrawPoint(x-1,y-1);
	LCD_Draw_Circle(x,y,6);//������Ȧ
}	  
//��һ�����(2*2�ĵ�)		   
//x,y:����
//color:��ɫ
void TP_Draw_Big_Point(u16 x,u16 y,u16 color)
{	    
	LCD_brush_color=color;
	LCD_DrawPoint(x,y);//���ĵ� 
	LCD_DrawPoint(x+1,y);
	LCD_DrawPoint(x,y+1);
	LCD_DrawPoint(x+1,y+1);	 	  	
}	

//��ʾУ׼���(��������)
void TP_Adj_Info_Show(u16 x0,u16 y0,u16 x1,u16 y1,u16 x2,u16 y2,u16 x3,u16 y3,u16 fac)
{	  
	LCD_brush_color=RED;
	LCD_string_font=ASCII_0816;
	LCD_Display_String(40,160,"x1:");
 	LCD_Display_String(40+80,160,"y1:");
 	LCD_Display_String(40,180,"x2:");
 	LCD_Display_String(40+80,180,"y2:");
	LCD_Display_String(40,200,"x3:");
 	LCD_Display_String(40+80,200,"y3:");
	LCD_Display_String(40,220,"x4:");
 	LCD_Display_String(40+80,220,"y4:");  
 	LCD_Display_String(40,240,"fac is:");     
	LCD_Display_Format(40+24,160,"%4d",x0);		//��ʾ��ֵ
	LCD_Display_Format(40+24+80,160,"%4d",y0);	//��ʾ��ֵ
	LCD_Display_Format(40+24,180,"%4d",x1);		//��ʾ��ֵ
	LCD_Display_Format(40+24+80,180,"%4d",y1);	//��ʾ��ֵ
	LCD_Display_Format(40+24,200,"%4d",x2);		//��ʾ��ֵ
	LCD_Display_Format(40+24+80,200,"%4d",y2);	//��ʾ��ֵ
	LCD_Display_Format(40+24,220,"%4d",x3);		//��ʾ��ֵ
	LCD_Display_Format(40+24+80,220,"%4d",y3);	//��ʾ��ֵ
 	LCD_Display_Format(40+56,240,"%3d",fac); 	//��ʾ��ֵ,����ֵ������95~105��Χ֮��.

}

//������У׼����
//�õ��ĸ�У׼����
double fac; 
u16 d1,d2;
void TP_Adjust(void)
{								 
	u16 pos_temp[4][2];//���껺��ֵ
	uint8_t  cnt=0;	

	u32 tem1,tem2;
	
 	cnt=0;				
	LCD_Clear(WHITE);//����   
	LCD_board_color =WHITE;
	LCD_brush_color=RED;
	LCD_chinese_font=Chinese_2424;
//	LCD_string_font=ASCII_0816;
//	LCD_Display_Message(256,100, "A");
	LCD_Display_Chinese(256,70, "����һֱ�ƶ�ֱ��У׼�ɹ�");//��ʾ��ʾ��Ϣ
	TP_Drow_Touch_Point(20,20,RED);//����1 
	tp_dev.sta=0;//���������ź� 
	tp_dev.xfac=0;//xfac��������Ƿ�У׼��,����У׼֮ǰ�������!�������	 
	while(1)//�������10����û�а���,���Զ��˳�
	{
		LCD_Display_Chinese(280,40,  "������Ļ�ϳ��ֵĹ��");//��ʾ��ʾ��Ϣ
		my_delayms(100);
		tp_dev.scan(1);//ɨ����������
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
		{		
			tp_dev.sta&=~TP_CATH_PRES;//��ǰ����Ѿ����������.
						   			   
			pos_temp[cnt][0]=tp_dev.x;
			pos_temp[cnt][1]=tp_dev.y;
			cnt++;	  
			switch(cnt)
			{			   
				case 1:						 
					TP_Drow_Touch_Point(20,20,WHITE);				//�����1 
					TP_Drow_Touch_Point(lcddev.width-20,20,RED);	//����2
					break;
				case 2:
 					TP_Drow_Touch_Point(lcddev.width-20,20,WHITE);	//�����2
					TP_Drow_Touch_Point(20,lcddev.height-20,RED);	//����3
					break;
				case 3:
 					TP_Drow_Touch_Point(20,lcddev.height-20,WHITE);			//�����3
 					TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,RED);	//����4
					break;
				case 4:	 //ȫ���ĸ����Ѿ��õ�
	    		    //�Ա����
					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);//x1-x2
					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);//y1-y2
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,2�ľ���
					
					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);//x3-x4
					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);//y3-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�3,4�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05||d1==0||d2==0)//���ϸ�
					{
						cnt=0;
						TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
						TP_Drow_Touch_Point(20,20,RED);								//����1
// 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//��ʾ����   
 						continue;
					}
					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,3�ľ���
					
					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�2,4�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//���ϸ�
					{
						cnt=0;
						TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
						TP_Drow_Touch_Point(20,20,RED);								//����1
// 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//��ʾ����   
						continue;
					}//��ȷ��
								   
					//�Խ������
					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//�õ�1,4�ľ���
	
					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//�õ�2,3�ľ���
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//���ϸ�
					{
						cnt=0;
						TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
						TP_Drow_Touch_Point(20,20,RED);								//����1
// 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//��ʾ����   
						continue;
					}//��ȷ��
					//������
					tp_dev.xfac=(float)(lcddev.width-40)/(pos_temp[1][0]-pos_temp[0][0]);//�õ�xfac		 
					tp_dev.xoff=(lcddev.width-tp_dev.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;//�õ�xoff
						  
					tp_dev.yfac=(float)(lcddev.height-40)/(pos_temp[2][1]-pos_temp[0][1]);//�õ�yfac
					tp_dev.yoff=(lcddev.height-tp_dev.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;//�õ�yoff  
					if(fabs(tp_dev.xfac)>2||fabs(tp_dev.yfac)>2)//������Ԥ����෴��.
					{
						cnt=0;
						TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//�����4
						TP_Drow_Touch_Point(20,20,RED);								//����1
						tp_dev.touchtype=!tp_dev.touchtype;//�޸Ĵ�������.
						if(tp_dev.touchtype)//X,Y��������Ļ�෴
						{
							TP_CMD_RDX =0X90;
							TP_CMD_RDY=0XD0;	 
						}else				   //X,Y��������Ļ��ͬ
						{
							TP_CMD_RDX=0XD0;
							TP_CMD_RDY=0X90;	 
						}			    
						continue;
					}		
					LCD_Clear(WHITE);//����
					LCD_brush_color=BLUE;
					LCD_board_color=WHITE;
					LCD_chinese_font=Chinese_2424;
					LCD_Display_Chinese(355,220,"��ĻУ׼���");//У�����
					my_delayms(1000);
					TP_Save_Adjdata();  
 					LCD_Clear(WHITE);//����   
					return;//У�����				 
			}
		}
 	}
}	 


//�жϴ������Ƿ���ĳһ������
uint8_t TP_Area_Press_Judge(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey)
{
	if(tp_dev.x>sx && tp_dev.x<ex && tp_dev.y>sy && tp_dev.y<ey)
		return 1;
	else
		return 0;
}

//��������ʼ��  		    
//����ֵ:	0,û�н���У׼
//       	1,���й�У׼
uint8_t TP_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	TP_CS_H();
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TP_IRQ_DISABLE();
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	TP_Read_XY(&tp_dev.x,&tp_dev.y);//��һ�ζ�ȡ��ʼ��
	if(TP_Get_Adjdata())
		return 0;//��У׼
	else			 //δУ׼
	{ 										    
		LCD_Clear(WHITE);	//����
		TP_Adjust();  		//��ĻУ׼  
	}			
	TP_Get_Adjdata();	
	
	return 1; 									 
}


void EXTI1_IRQHandler(void)
{
	if ( EXTI_GetITStatus(EXTI_Line1) != RESET )
	{
		TP_Scan(0);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}		
}
