//#include "Con_Phoenix.h"
#include "sys.h"
#include "colorcfg.h"
#include "lcd.h"
#include "ov7670.h" 
#include "ov7725.h"
#include "delay.h"
#include "key.h"
//����OV7725��������װ��ʽԭ��,OV7725_WINDOW_WIDTH�൱��LCD�ĸ߶ȣ�OV7725_WINDOW_HEIGHT�൱��LCD�Ŀ��
//ע�⣺�˺궨��ֻ��OV7725��Ч
#define  OV7725_WINDOW_WIDTH		240 // <=320		LCD��ʾ��ͼ��ߴ硪���߶�
#define  OV7725_WINDOW_HEIGHT		320 // <=240		LCD��ʾ��ͼ��ߴ硪�����
#define  OV7725 1			
#define  OV7670 2			
extern u8 ov_sta;		//��exit.c�� �涨�塪��֡�жϱ�ǣ��������ͼ�����ݴ�����ٶȡ�
//extern u8 ov_frame;	//��timer.c���涨�塪������ͳ��֡�����Դ�ӡ֡��
extern u8 sensor;
extern  u8 color_list[COLOR_NUM][7+7];
extern  u8 object_flag;
#define mode_colo  1






///1.///////////////////////////////////////////////////////////////////////////
/*
	����LCD��ʾ(OV7725)
	��֡�жϿ����£���FIFO��������ݴ��ݵ�MCU���沢��ʾ��LCD����ȥ
*/ 
void OV7725_camera_refresh(void)
{
	u32 i,j;
 	u16 color;
	u16 gray;	

u16 yuzhi=60;  //����ֵ����ֵ������ǿ��ֵҪ���õ�С������Ŀ����0-200������  ����ԽСֵԽС

	
//	LCD_Fill(0,230,480,570,		BLACK);	//	 ��Ҫ��Ϊ�������� ��ϣ�����ϣ��벿���ǰ�ɫ����Ѹ���ע�͵����ɣ����߻�����������ɫ	
	if(ov_sta)//��֡�жϸ��£�
	{
		LCD_Scan_Dir(U2D_L2R);		//���ϵ���,������  
		if(lcddev.id==0X1963)LCD_Set_Window((lcddev.width-240)/2,(lcddev.height-320)/2,240,320);//����ʾ�������õ���Ļ����
		else if(lcddev.id==0X5510||lcddev.id==0X5310)LCD_Set_Window((lcddev.width-320)/2,(lcddev.height-240)/2,320,240);//����ʾ�������õ���Ļ����
		LCD_WriteRAM_Prepare();     //��ʼд��GRAM	
		OV7725_RRST=0;				//��ʼ��λ��ָ�� 
		OV7725_RCK_L;
		OV7725_RCK_H;
		OV7725_RCK_L;
		OV7725_RRST=1;				//��λ��ָ����� 
		OV7725_RCK_H; 
	for(i=0;i<OV7725_WINDOW_WIDTH;i++)
		{
			for(j=0;j<OV7725_WINDOW_HEIGHT;j++)
			{
			OV7725_RCK_L;
			color=OV7670_DATA;	//������
			OV7725_RCK_H; 
			color<<=8;  
			OV7725_RCK_L;
			color|=OV7670_DATA;	//������
			OV7725_RCK_H; 
			gray=(((color&0xF800)>>8)*19595+((color&0x07E0)>>3)*38469 +((color&0x1f)<<3)*7472)>>16;    //ͼ���ֵ��
			if(gray>=yuzhi) LCD->LCD_RAM=WHITE ;
				else     LCD->LCD_RAM=BLACK ;     
			}
		}
//     if(Trace(&Conditionred,&Resured))                      //API
//			{				
//			 LCD_Fillred(Resured.x-Resured.w/2,Resured.y-Resured.h/2,Resured.x+Resured.w/2,Resured.y-Resured.h/2+1,0xf800);//u16 x,u16 y,u16 width,u16 hight,u16 Color
//				LCD_Fillred(Resured.x-Resured.w/2,Resured.y-Resured.h/2,Resured.x-Resured.w/2+1,Resured.y+Resured.h/2,0xf800);
//				LCD_Fillred(Resured.x-Resured.w/2,Resured.y+Resured.h/2,Resured.x+Resured.w/2,Resured.y+Resured.h/2+1,0xf800);
//				LCD_Fillred(Resured.x+Resured.w/2,Resured.y-Resured.h/2,Resured.x+Resured.w/2+1,Resured.y+Resured.h/2,0xf800);
//				LCD_Fillred(Resured.x-2,Resured.y-2,Resured.x+2,Resured.y+2,0xf800);
//	
//				r=Resured.x;
//			  y=Resured.y;
//			}	
		
 		ov_sta=0;					//����֡�жϱ�
		LCD_Scan_Dir(DFT_SCAN_DIR);	//�ָ�Ĭ��ɨ�跽��
//		LED1=!LED1;		
	} 
}
 u16 yuzhi=40;  
void OV7725_camera_refresh_color_track(void)
{
	u32 i,j;
 	u16 color;
	u16 gray;	

 
//											LCD_Fill(0,230,480,570,		BLACK);	//��ϣ�����ϣ��벿���ǰ�ɫ����Ѹ���ע�͵����ɣ����߻�����������ɫ		
	if(ov_sta)//��֡�жϸ��£�//ֻҪ����EXTI�ж�,�ñ������Լӷ��㣬��ô�������ͻ��жϵ���������,�Ӷ�ִ�б�����(ͼ�����ݶ�ȡ����)
	{//��"(ͼ��ɼ�����)"��"(ͼ�����ݶ�ȡ����)"ͬʱ����,����������ͼ��������ȫ����FIFO֮����ȥ��,�Ӷ������ͼ�����ݴ�����ٶȡ�
		LCD_Scan_Dir(U2D_L2R);//���ϵ���,������
		LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT);//����ʾ�������õ���Ļ����
		if(lcddev.id==0X1963)
        LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_HEIGHT,OV7725_WINDOW_WIDTH);//����ʾ�������õ���Ļ����
		LCD_WriteRAM_Prepare();     //��ʼд��GRAM	
		OV7725_RRST=0;				//��ʼ��λ��ָ�� 
		OV7725_RCK_L;
		OV7725_RCK_H;
		OV7725_RCK_L;
		OV7725_RRST=1;				//��λ��ָ����� 
		OV7725_RCK_H; 
		for(i=0;i<OV7725_WINDOW_WIDTH;i++)
		{
			for(j=0;j<OV7725_WINDOW_HEIGHT;j++)
			{
				OV7725_RCK_L;
				color=OV7725_DATA;	//������
				OV7725_RCK_H; 
				color<<=8;  
				OV7725_RCK_L;
				color|=OV7725_DATA;	//������
				OV7725_RCK_H; 

					gray=(((color&0xF800)>>8)*19595+((color&0x07E0)>>3)*38469 +((color&0x1f)<<3)*7472)>>16;    //ͼ���ֵ��
			if(gray>=yuzhi) LCD->LCD_RAM=WHITE ;
				else     LCD->LCD_RAM=BLACK ;  

				
			}
		}
 		ov_sta=0;	//����֡�жϱ��,  ʹ֮������ѭ��ִ��"(ͼ��ɼ�����)����"��"(ͼ�����ݶ�ȡ����)����"
//		ov_frame++; //֡�ж�, ֡������, ����printf����֡������ӡ��ȥ��
		LCD_Scan_Dir(DFT_SCAN_DIR);	//�ָ�Ĭ��ɨ�跽��Ĭ����"������,���ϵ���"ɨ�跽ʽ,���ȵ�һ���ٵڶ��е�����...��  
	} 
}

/*
	����LCD��ʾ(OV7670)
	��֡�жϿ����£���FIFO��������ݴ��ݵ�MCU���沢��ʾ��LCD����ȥ
*/
void OV7670_camera_refresh(void)
{
	u32 j;
 	u16 color;	 
	if(ov_sta)//��֡�жϸ���
	{
		LCD_Scan_Dir(U2D_L2R);//���ϵ���,������  
		if(lcddev.id==0X1963)LCD_Set_Window((lcddev.width-240)/2,(lcddev.height-320)/2,240,320);//����ʾ�������õ���Ļ����
		else if(lcddev.id==0X5510||lcddev.id==0X5310)LCD_Set_Window((lcddev.width-320)/2,(lcddev.height-240)/2,320,240);//����ʾ�������õ���Ļ����
		LCD_WriteRAM_Prepare();     //��ʼд��GRAM	
		OV7670_RRST=0;				//��ʼ��λ��ָ�� 
		OV7670_RCK_L;
		OV7670_RCK_H;
		OV7670_RCK_L;
		OV7670_RRST=1;				//��λ��ָ����� 
		OV7670_RCK_H;
		for(j=0;j<76800;j++)
		{
			OV7670_RCK_L;
			color=GPIOC->IDR&0XFF;	//������
			OV7670_RCK_H; 
			color<<=8;  
			OV7670_RCK_L;
			color|=GPIOC->IDR&0XFF;	//������
			OV7670_RCK_H; 
			LCD->LCD_RAM=color;    
		}   							  
 		ov_sta=0;					//����֡�жϱ��
//		ov_frame++; 
		LCD_Scan_Dir(DFT_SCAN_DIR);	//�ָ�Ĭ��ɨ�跽�� 
	} 
}

/*
	OV7725��ʼ��||7670��ʼ��
*/
void Ov7725_7670_Choose_Init(void)
{
	u8 lightmode=0,effect=0;
	s8 saturation=0,brightness=0,contrast=0;
	
	POINT_COLOR=RED;		 
  LCD_ShowString(30,10,200,16,16,"OV7725_OV7670 Init...");	
	
	while(1)//��ʼ��OV7725_OV7670
	{
		if(OV7725_Init()==0)//7725��ʼ��
		{
			sensor=OV7725;
			LCD_ShowString(30,10,200,16,16,"OV7725 Init OK       ");
			
//			OV7725_Window_Set(OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT,1);//QVGAģʽ���
			
			OV7725_Light_Mode(lightmode);
			OV7725_Color_Saturation(saturation);
			OV7725_Brightness(brightness);
			OV7725_Contrast(contrast);
			OV7725_Special_Effects(effect);
			OV7725_CS=0;//һֱʹ��
			break;
		}
		else if(OV7670_Init()==0)//7670��ʼ��
		{
			sensor=OV7670;
			LCD_ShowString(30,230,200,16,16,"OV7670 Init OK       ");
			delay_ms(1500);	 	   
			OV7670_Light_Mode(lightmode);
			OV7670_Color_Saturation(saturation);
			OV7670_Brightness(brightness);
			OV7670_Contrast(contrast);
			OV7670_Special_Effects(effect);
			OV7670_Window_Set(12,176,240,320);//���ô���	"240,320"���û�����(��������Ҫ���õķֱ���)  
			//"12,176"�ǹٷ�Դ������,�����޸�;���޸ĵ���ֵƫ�����ֵ�Ƚ�Զ�Ļ�,��ᷢ��ͼ��ᱻ�ָ��Ӱ�졣
			OV7670_CS=0;//һֱʹ��
			break;
		}
		else
		{
			LCD_ShowString(30,230,200,16,16,"OV7725_OV7670 Error!!");
			delay_ms(200);
			LCD_Fill(30,230,239,246,WHITE);
			delay_ms(200);
		}
	}
}

/*
	OV7725������ʾ||7670������ʾ
*/
void OV7725_7670_Camera_Refresh_Phoenix(void)
{
	if(sensor==OV7725)					OV7725_camera_refresh();//7725������ʾ
	else if(sensor==OV7670)		OV7670_camera_refresh();//7670������ʾ	
}






///2.///////////////////////////////////////////////////////////////////////////
/*
	�����޸ĺ���������������ָ����
*/
//void Key_Modify_Phoenix(void)
//{
//	
//	u8 key; 
//	key=KEY_Scan(1);
//	if(key==WKUP_PRES)	color_assignment();//��ͼ��ģ�5��5������ɫ����Ϊ�µ�ʶ����ɫ��ʹ�ø÷��������֡�ʣ�
//	if(key==KEY2_PRES)	PID.Position_KP+=0.1; 	//���ñ���ϵ��			Kp	
//	if(key==KEY1_PRES)	PID.Position_KI+=0.1;		//���û���ʱ�䳣��	Ki
//	if(key==KEY0_PRES)	PID.Position_KD+=0.1;		//����΢��ʱ�䳣��	Kd
//}





///3.///////////////////////////////////////////////////////////////////////////
/*
	��ͼ��ģ�5��5������ɫ����Ϊ�µ�ʶ����ɫ
*/
//void color_assignment(void)
//{  
//	COLOR_RGB_t rgb_tmp;
//	COLOR_HLS_t hls_tmp;

//	ReadColor(IMG_X+5,IMG_Y+5, &rgb_tmp);//��ͼ���rgb

//	RGB2HSL( &rgb_tmp, &hls_tmp );//ת����hsl

//	condition[global_page].H_MIN=hls_tmp.Hue-COLOR_RANG;
//	condition[global_page].H_MAX=hls_tmp.Hue+COLOR_RANG;

//	condition[global_page].L_MIN=hls_tmp.Lightness-COLOR_RANG;
//	condition[global_page].L_MAX=hls_tmp.Lightness+COLOR_RANG;

//	condition[global_page].S_MIN=hls_tmp.Saturation-COLOR_RANG;
//	condition[global_page].S_MAX=hls_tmp.Saturation+COLOR_RANG;

//	condition[global_page].HEIGHT_MIN=40;
//	condition[global_page].HEIGHT_MAX=400;

//	condition[global_page].WIDTH_MAX=400;
//	condition[global_page].WIDTH_MIN=40;


////	/*RGB������ʾ*/
////	LCD_ShowxNum(100,600,rgb_tmp.Red,3,24,0x80);							//RGB--Red		����
////	LCD_ShowxNum(100+12+12*3,600,rgb_tmp.Green,3,24,0x80);		//RGB--Green	����
////	LCD_ShowxNum(100+12*2+12*6,600,rgb_tmp.Blue,3,24,0x80);		//RGB--Blue		����

////	/*HSL������ʾ*/
////	LCD_ShowxNum(100,630,hls_tmp.Hue,3,24,0x80);									//HSL--	Hue					data
////	LCD_ShowxNum(100+12+12*3,630,hls_tmp.Saturation,3,24,0x80);		//HSL--	Saturation	data
////	LCD_ShowxNum(100+12*2+12*6,630,hls_tmp.Lightness,3,24,0x80);	//HSL--	Lightness		data

////	/*H_data_MAX&MIN��ʾ*/
////	LCD_ShowxNum(100,660,condition[global_page].H_MIN,3,24,0x80);					//H_MIN
////	LCD_ShowxNum(100+12+12*3,660,condition[global_page].H_MAX,3,24,0x80);	//H_MAX


//	LCD_ShowString((IMG_X+IMG_W)/2,(IMG_Y+IMG_H)/2,200,16,16,(u8 *)"set complete!!");
//	if(++global_page>=COLOR_NUM)
//			global_page=0;
//}


/*
	��ӡ	��Ŀ��ɫ�����ĵ㣨ʮ�����ĵ㣩�����в���	��LCD
object_num  ����������������ǰ�Ѿ�ʶ���˶��ٸ����壬Ȼ�����ѡ����ʾ��Ӧ���ַ�
*/
void color_Value(u16 x,u16 y,u8 object_num)
{
    COLOR_RGB_t rgb_tmp;
    COLOR_HLS_t hls_tmp;

  ReadColor(x+1,y+1,&rgb_tmp);//��ͼ���rgb 


    RGB2HSL(&rgb_tmp,&hls_tmp);//ת����hsl
		

		LCD_ShowxNum(186,600,rgb_tmp.Red,3,24,0x80);
		LCD_ShowxNum(186+12*6,600,rgb_tmp.Green,3,24,0x80);
		LCD_ShowxNum(186+12*12,600,rgb_tmp.Blue,3,24,0x80);
		
		LCD_ShowxNum(186,630,hls_tmp.Hue,3,24,0x80);
		LCD_ShowxNum(186+12*6,630,hls_tmp.Saturation,3,24,0x80);
		LCD_ShowxNum(186+12*12,630,hls_tmp.Lightness,3,24,0x80);
	
		LCD_ShowxNum(186,660,x,3,24,0x80);
		LCD_ShowxNum(186+12*6,660,y,3,24,0x80);
	
		if(result[object_num].object==0)     							//��ʾʶ�������
			{
					LCD_ShowString(30,100,480,24,24,	"Object is round          ");   //Բ��
			}
			else
						if(result[object_num].object==1)
			{
					LCD_ShowString(30,100,480,24,24,	"Object is rectangle      ");		//����
			}
}
/*
�ҵ��Ե�ʱ���õģ����ż�������Բ����ʾ���������Ҫȥ�˽�Bresenham�㷨��Բ
*/
void Draw_1(u16 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //�ж��¸���λ�õı�־
	while(a<=b)
	{
 	         
		a++;
		//ʹ��Bresenham�㷨��Բ     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}


			draw_cross(x0-a-5, y0-b-6);//��ʮ��׼��	
			draw_cross(x0+a+5, y0-b-6);//��ʮ��׼��


			draw_cross(x0-a-5, y0+b+6);//��ʮ��׼��
			draw_cross(x0+a+5, y0+b+6);//��ʮ��׼��
} 


/*
	����ʶ��
condition[0] 	��ɫ����ʶ��
result[i_2]		������������
*/
u8 data;
u8 xianshi[1];
void Feature_Recognize_Start_Phoenix(void)
{
	u8 i_2=0,flag=0,count_color=0,distinguish=0;
  static u8	j=0,count_spot=0;

		for(j=0;j<2;j++)																									//ʶ���Ϊ�����֣�һ������������״ʶ��һ������������ɫʶ��
		{
		if(!j)																														//j==0Ϊ��״ʶ��  j==1Ϊ��״��ɫʶ��
		{
			for(i_2=0;i_2<OBJECT_NUM;i_2++)  																//OBJECT_NUM  ��colorcfg.h����ĺ궨�壬���嵱ǰ����ʶ�����������
	    {
//			LCD_ShowString(30+120,50,480,24,24,			"Searching");						//��һ����ʾ����ʶ������Ĺ����У���ɫ��ʶ�����Ը���һ��search����ʾ

		if(Trace(&condition[0], &result[i_2], &area[i_2],i_2)&&j==0)			//ִ����ɫʶ��  �������Ρ�����ʮ��׼�ǡ�// trace�����٣�ʶ����ɫ���� condition��ʶ����ɫ������һ���ṹ����������洢��HLS��threshold��result����������ʶ�𵽵������������С
		{ 
//     close_EXTI8_Init();				
			data=Digital_recognition(result[i_2].Xmin,result[i_2].Xmax,result[i_2].Ymin,result[i_2].Ymax,condition,result[i_2].w,result[i_2].h);

			LCD_DrawRectangle( result[i_2].Xmin, result[i_2].Ymin, result[i_2].Xmax,  result[i_2].Ymax);//������                     
			draw_cross(result[i_2].x, result[i_2].y);												//��ʮ��׼��
       USART_SendData(USART2,data);
			xianshi[0]=data+48;
			LCD_ShowString(200,280,200,16,16,&xianshi[0]);
			//      LCD_Draw_Circle(result[i_2].x,result[i_2].y,result[i_2].h/2);
//			color_Value(result[i_2].x,result[i_2].y,i_2);										//��Ŀ������ĵ�(�ӽ����ĵ�)����ɫ�Ĳ�����ӡ��LCD��		
			flag=1;																													//ʶ�������壬���һ�£�������ɫʶ��������flagΪ1ʱ���Ž�����ɫʶ��Ҳ����˵����ǰ�Ѿ�ʶ�������壬û��ʶ���������ý����������ɫʶ��
//			distinguish+=1;																									//��¼ʶ������ĸ������������ɫʶ����õ�
		  delay_ms(10);																									//��ʱһ�ºù۲�

//			LCD_Fill(result[i_2].x-result[i_2].w/2-3,result[i_2].y-result[i_2].h/2-6,result[i_2].x-result[i_2].w/2+result[i_2].w+8,  result[i_2].y-result[i_2].h/2+result[i_2].h+8,BLACK);    //����
			
			delay_ms(6);																									//��ʱһ�ºù۲�
//	EXTI8_Init();
			count_spot=0;																										//һ�����������										
		}	
		
//		else																															//���û�±����壬����ʾ���������͵��Ķ�����ʾ
//			{
////				LCD_ShowString(30+120,100,480,24,24,		"Searching");

////				if(count_spot==0)
////				{
////				  LCD_ShowString(30+19*12,100,480,24,24,		".    		");
////					LCD_ShowString(30+120+9*12,50,480,24,24,	".    		");}
////				if(count_spot==1)
////				{
////				  LCD_ShowString(30+19*12,100,480,24,24,		"..   		");
////					LCD_ShowString(30+120+9*12,50,480,24,24,	"..   		");}
////				if(count_spot==2)
////				{
////				  LCD_ShowString(30+19*12,100,480,24,24,		"...  		");
////					LCD_ShowString(30+120+9*12,50,480,24,24,	"...  		");}
////				if(count_spot==3)
////				{
////				  LCD_ShowString(30+19*12,100,480,24,24,		"....    	");
////					LCD_ShowString(30+120+9*12,50,480,24,24,	"....    	");}
////				if(count_spot==4)
////				{
////				  LCD_ShowString(30+19*12,100,480,24,24,		".....   	");
////					LCD_ShowString(30+120+9*12,50,480,24,24,	".....   	");
////					count_spot=0;
////				}
//				  count_spot++; 
//			}
	}	
		

}		
/*
distinguish ʶ�𵽵��������
�ж�������ͽ��ж��ٴ���ɫʶ��
COLOR_NUM    ��ɫ���������������Ǵ�����1��ʼ����Ϊǰ����Ǹ�0����������ʶ���õ��ģ���ɫʶ���ã�����0������ɫ�����ﲻ��
color_list   ��ɫ�ı��б� ���� 
*/
		
//else																																//ʶ�������壬������ɫʶ��
//	if(j&&flag)																												//j==0Ϊ��״ʶ��  j==1Ϊ��״��ɫʶ��
//{

//		OV7725_camera_refresh_color_track();														//ͼƬ��ʾ����������ֵ������
//		for(count_color=0;count_color<distinguish;count_color++)
//	{

//			for(i_2=1;i_2<COLOR_NUM;i_2++)																//�ж�������ͽ��ж��ٴ���ɫʶ��
//		{
//				if(consend_color(&condition[i_2],&result[count_color],&Quadrant_control ))	//��ɫʶ����
//			{
////					LCD_ShowString(30+120,50,480,24,24,	&color_list[i_2-1][0]);								//���ʶ���ˣ�����LCD����ʾ��ɫ�ı�����
//					break;
//			}

//		}
//				if(i_2>COLOR_NUM-1)	                                                     		//������˵�����ʶ�����������������ɫ�ĸ�����˵��û��ʶ����Ӧ����ɫ
//				LCD_ShowString(30+120,50,480,24,24,"other              "); 
//	    	color_Value(result[count_color].x,result[count_color].y,count_color);				//��Ŀ������ĵ�(�ӽ����ĵ�)����ɫ�Ĳ�����ӡ��LCD��		

//				LCD_DrawRectangle( result[count_color].x-result[count_color].w/2, result[count_color].y-result[count_color].h/2, result[count_color].x-result[count_color].w/2+result[count_color].w,  result[count_color].y-result[count_color].h/2+result[count_color].h);//������                     
//			
//		    draw_cross(result[count_color].x, result[count_color].y);										//��ʮ��׼��
//	
////		    LCD_Draw_Circle(result[count_color].x,result[count_color].y,result[count_color].h/2);  //��Բ
//		
//				flag=0;																																			//���ʶ���־
//	     delay_ms(10);																															//��ʱһ�£��ù۲�
//		}
////				LCD_ShowString(30+120,100,480,24,24,		 "                  ");							//�����ַ��õ�
////				LCD_ShowString(30+120,50,480,24,24,	     "                  ");		
//	}
	
}
}







///4.///////////////////////////////////////////////////////////////////////////
/*
	PID����
*/
//void PID_LR_figure_Phoenix(void)
//{
//	/*1.��ȡƫ��*/
//	PID.Bias_LR=PID.Feedback_X-PID.User_X;

//	/*2.����PID*/
//	PID.PID_LR_current_out=Position_PID(PID.Feedback_X,PID.User_X);//User_X����������    Feedback_X������ǰ����

//	/*3.�������ȡ����ֵ*/
//	PWM_abs(&PID.PID_LR_current_out);//ȡ����ֵ

//	/*4.����*/
//	if(PID.Bias_LR>0){PID.PWM_value_LR+=PID.PID_LR_current_out;}//���ң������ң�������7725���ų��·��ã����еġ�+=����Ϊ��-=����
//	if(PID.Bias_LR<0){PID.PWM_value_LR-=PID.PID_LR_current_out;}//���󣬵����󣬼�С����7725���ų��·��ã����еġ�-=����Ϊ��+=����

//	/*5.PWM�޷�*/
//	Xianfu_Pwm(&PID.PWM_value_LR);//�޷�PWM

//	/*6.����PWM*/
//	TIM_SetCompare1(TIM3,PID.PWM_value_LR);
//}

///*
//	PID����
//*/
//void PID_UD_figure_Phoenix(void)
//{
//	/*1.��ȡƫ��*/
//	PID.Bias_UD=PID.Feedback_Y-PID.User_Y;

//	/*2.����PID*/
//	PID.PID_UD_current_out=Position_PID(PID.Feedback_Y,PID.User_Y);

//	/*3.�������ȡ����ֵ*/
//	PWM_abs(&PID.PID_UD_current_out);//ȡ����ֵ

//	/*4.����*/
//	if(PID.Bias_UD>0){PID.PWM_value_UD-=PID.PID_UD_current_out;}//���£������£���С����7725���ų��·��ã����еġ�-=����Ϊ��+=����
//	if(PID.Bias_UD<0){PID.PWM_value_UD+=PID.PID_UD_current_out;}//���ϣ������ϣ�������7725���ų��·��ã����еġ�+=����Ϊ��-=����

//	/*5.PWM�޷�*/
//	Xianfu_Pwm(&PID.PWM_value_UD);//�޷�PWM

//	/*6.����PWM*/
//	TIM_SetCompare2(TIM3,PID.PWM_value_UD);
//}

///*
//	PID��Ӧ������ֵ��ʼ��
//*/
//void Constant_Init(void)
//{
//	PID.Position_KP=0;			PID.Position_KI=0;	PID.Position_KD=0;	//����λ��PID��������PID����
//	PID.Increment_KP=0;			PID.Increment_KI=0;											//��������PI��������PI����
//	PID.Bias_LR=0;					PID.Bias_UD=0;													//����ƫ��Bias_LR:����LR:leftright������ƫ��Bias_UD:����UD:updown
//	PID.User_X=240;					PID.User_Y=400;													//�û�����ֵ
//	PID.Feedback_X=0;				PID.Feedback_Y=0;												//����ֵ	
//	
//	PID.PWM_value_LR=1400,	PID.PWM_value_UD=1400;									//����PWM���ռ���ֵ--����LR:leftright		����PWM���ռ���ֵ--����UD:updown
//}



///*
//	���ڲ��Զ���Ƿ�����
//	����(-90��)��>(0��)��>(+90��)
//*/
//void PWM_TEST(void)
//{
//	TIM_SetCompare1(TIM3,500);	TIM_SetCompare2(TIM3,500);	delay_ms(500);	
//	TIM_SetCompare1(TIM3,1400);	TIM_SetCompare2(TIM3,1400);	delay_ms(500);
//	TIM_SetCompare1(TIM3,2200);	TIM_SetCompare2(TIM3,2200);	delay_ms(500);	
//}



