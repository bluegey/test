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
//默认为touchtype=0的数据.
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
//SPI写数据
//向触摸屏IC写入1byte数据    
//num:要写入的数据
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
		TP_SCK_H();		//上升沿有效	        
	}		 			    
}*/

//SPI读数据 
//从触摸屏IC读取adc值
//CMD:指令
//返回值:读到的数据	   
u16 TP_Read_AD(uint8_t CMD)	  
{ 	   
	u16 Num=0; 
	
	TP_CS_L();  	//选中触摸屏IC
	TP_SPI_RW_Byte(CMD);//发送命令字
	my_delayus(6);//ADS7846的转换时间最长为6us 	    
	Num=TP_SPI_RW_Byte(0x00);
	Num=Num<<8;	
	Num|=TP_SPI_RW_Byte(0x00);
	Num>>=3;   	
	TP_CS_H();		//释放片选	 
	return(Num);   
}

//读取一个坐标值(x或者y)
//连续读取READ_TIMES次数据,对这些数据升序排列,
//然后去掉最低和最高LOST_VAL个数,取平均值 
//xy:指令（CMD_RDX/CMD_RDY）
//返回值:读到的数据
#define READ_TIMES 5 	//读取次数
#define LOST_VAL 1	  	//丢弃值
u16 TP_Read_XOY(uint8_t xy)
{
	u16 i, j;
	u16 buf[READ_TIMES];
	u16 sum=0;
	u16 temp;
	for(i=0;i<READ_TIMES;i++)
		buf[i]=TP_Read_AD(xy);		 		    
	for(i=0;i<READ_TIMES-1; i++)//排序
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//升序排列
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

//读取x,y坐标
//x,y:读取到的坐标值
//返回值:0,失败;1,成功。
uint8_t TP_Read_XY(u16 *x,u16 *y)
{
	u16 xtemp,ytemp;			 	 		  
	xtemp=TP_Read_XOY(TP_CMD_RDX);
	ytemp=TP_Read_XOY(TP_CMD_RDY);	  												   
	
	*x=xtemp;
	*y=ytemp;
	return 1;//读数成功
}

//连续2次读取触摸屏IC,且这两次的偏差不能超过
//ERR_RANGE,满足条件,则认为读数正确,否则读数错误.	   
//该函数能大大提高准确度
//x,y:读取到的坐标值
//返回值:0,失败;1,成功。
#define ERR_RANGE 50 //误差范围 
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
	if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//前后两次采样在+-50内
	&&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
	{
			*x=(x1+x2)/2;
			*y=(y1+y2)/2;
			return 1;
	}
	else 
		return 0;	  
} 

//触摸按键扫描
//tp:0,屏幕坐标;1,物理坐标(校准等特殊场合用)
//返回值:当前触屏状态.
//0,触屏无触摸;1,触屏有触摸
uint8_t TP_Scan(uint8_t tp)
{			 

	if(TP_IRQ_Read()==0)//  ----------------------------																																																																															b有按键按下
	{
		if(tp)
			TP_Read_XY2(&tp_dev.x,&tp_dev.y);//读取物理坐标
		else if(TP_Read_XY2(&tp_dev.x,&tp_dev.y))//读取屏幕坐标
		{
//	 		tp_dev.x=tp_dev.x/4.50f-60;//将结果转换为屏幕坐标     tp_dev.xfac*   tp_dev.yfac*
//			tp_dev.y=tp_dev.y/7.80f-40;  
			tp_dev.x=tp_dev.xfac*tp_dev.x+tp_dev.xoff;//将结果转换为屏幕坐标
			tp_dev.y=tp_dev.yfac*tp_dev.y+tp_dev.yoff;  
	 	} 
		if((tp_dev.sta&TP_PRES_DOWN)==0)//之前没有被按下
		{		 
			tp_dev.sta=TP_PRES_DOWN|TP_CATH_PRES;//按键按下    	   			 
		}			   
	}
	else
	{
		if(tp_dev.sta&TP_PRES_DOWN)//之前是被按下的
		{
			tp_dev.sta&=~TP_PRES_DOWN;//标记按键松开	
		}
		else//之前就没有被按下
		{
			tp_dev.x=0xffff;
			tp_dev.y=0xffff;
		}	    
	}
	return tp_dev.sta&TP_PRES_DOWN;//返回当前的触屏状态
}	 

//保存校准参数										    
void TP_Save_Adjdata(void)
{
	tp_dev.adjust_flag=0xAA;
	AT24CXX_Write(0,(uint8_t*)&tp_dev.xfac,14);	//强制保存&tp_dev.xfac地址开始的14个字节数据
}

//得到保存在EEPROM里面的校准值
//返回值：1，成功获取数据
//        0，获取失败，要重新校准
uint8_t TP_Get_Adjdata(void)
{					  
	AT24CXX_Read(0,(uint8_t*)&tp_dev.xfac,14);//读取之前保存的校准数据 
	if(tp_dev.adjust_flag==0xAA)//触摸屏已经校准过了			   
 	{ 
		if(tp_dev.touchtype)//X,Y方向与屏幕相反
		{
			TP_CMD_RDX=0X90;
			TP_CMD_RDY=0XD0;	 
		}
		else				   //X,Y方向与屏幕相同
		{
			TP_CMD_RDX=0XD0;
			TP_CMD_RDY=0X90;	 
		}		 
		return 1;	 
	}
	return 0;
}



//画一个触摸点
//用来校准用的
//x,y:坐标
//color:颜色
void TP_Drow_Touch_Point(u16 x,u16 y,u16 color)
{
	LCD_brush_color=color;
	LCD_DrawLine(x-12,y,x+13,y);//横线
	LCD_DrawLine(x,y-12,x,y+13);//竖线
	LCD_DrawPoint(x+1,y+1);
	LCD_DrawPoint(x-1,y+1);
	LCD_DrawPoint(x+1,y-1);
	LCD_DrawPoint(x-1,y-1);
	LCD_Draw_Circle(x,y,6);//画中心圈
}	  
//画一个大点(2*2的点)		   
//x,y:坐标
//color:颜色
void TP_Draw_Big_Point(u16 x,u16 y,u16 color)
{	    
	LCD_brush_color=color;
	LCD_DrawPoint(x,y);//中心点 
	LCD_DrawPoint(x+1,y);
	LCD_DrawPoint(x,y+1);
	LCD_DrawPoint(x+1,y+1);	 	  	
}	

//提示校准结果(各个参数)
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
	LCD_Display_Format(40+24,160,"%4d",x0);		//显示数值
	LCD_Display_Format(40+24+80,160,"%4d",y0);	//显示数值
	LCD_Display_Format(40+24,180,"%4d",x1);		//显示数值
	LCD_Display_Format(40+24+80,180,"%4d",y1);	//显示数值
	LCD_Display_Format(40+24,200,"%4d",x2);		//显示数值
	LCD_Display_Format(40+24+80,200,"%4d",y2);	//显示数值
	LCD_Display_Format(40+24,220,"%4d",x3);		//显示数值
	LCD_Display_Format(40+24+80,220,"%4d",y3);	//显示数值
 	LCD_Display_Format(40+56,240,"%3d",fac); 	//显示数值,该数值必须在95~105范围之内.

}

//触摸屏校准代码
//得到四个校准参数
double fac; 
u16 d1,d2;
void TP_Adjust(void)
{								 
	u16 pos_temp[4][2];//坐标缓存值
	uint8_t  cnt=0;	

	u32 tem1,tem2;
	
 	cnt=0;				
	LCD_Clear(WHITE);//清屏   
	LCD_board_color =WHITE;
	LCD_brush_color=RED;
	LCD_chinese_font=Chinese_2424;
//	LCD_string_font=ASCII_0816;
//	LCD_Display_Message(256,100, "A");
	LCD_Display_Chinese(256,70, "光标会一直移动直到校准成功");//显示提示信息
	TP_Drow_Touch_Point(20,20,RED);//画点1 
	tp_dev.sta=0;//消除触发信号 
	tp_dev.xfac=0;//xfac用来标记是否校准过,所以校准之前必须清掉!以免错误	 
	while(1)//如果连续10秒钟没有按下,则自动退出
	{
		LCD_Display_Chinese(280,40,  "请点击屏幕上出现的光标");//显示提示信息
		my_delayms(100);
		tp_dev.scan(1);//扫描物理坐标
		if((tp_dev.sta&(TP_CATH_PRES|TP_PRES_DOWN))==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
		{		
			tp_dev.sta&=~TP_CATH_PRES;//标记按键已经被处理过了.
						   			   
			pos_temp[cnt][0]=tp_dev.x;
			pos_temp[cnt][1]=tp_dev.y;
			cnt++;	  
			switch(cnt)
			{			   
				case 1:						 
					TP_Drow_Touch_Point(20,20,WHITE);				//清除点1 
					TP_Drow_Touch_Point(lcddev.width-20,20,RED);	//画点2
					break;
				case 2:
 					TP_Drow_Touch_Point(lcddev.width-20,20,WHITE);	//清除点2
					TP_Drow_Touch_Point(20,lcddev.height-20,RED);	//画点3
					break;
				case 3:
 					TP_Drow_Touch_Point(20,lcddev.height-20,WHITE);			//清除点3
 					TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,RED);	//画点4
					break;
				case 4:	 //全部四个点已经得到
	    		    //对边相等
					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);//x1-x2
					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);//y1-y2
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,2的距离
					
					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);//x3-x4
					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);//y3-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到3,4的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05||d1==0||d2==0)//不合格
					{
						cnt=0;
						TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
						TP_Drow_Touch_Point(20,20,RED);								//画点1
// 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//显示数据   
 						continue;
					}
					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,3的距离
					
					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到2,4的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//不合格
					{
						cnt=0;
						TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
						TP_Drow_Touch_Point(20,20,RED);								//画点1
// 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//显示数据   
						continue;
					}//正确了
								   
					//对角线相等
					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,4的距离
	
					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到2,3的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//不合格
					{
						cnt=0;
						TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
						TP_Drow_Touch_Point(20,20,RED);								//画点1
// 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//显示数据   
						continue;
					}//正确了
					//计算结果
					tp_dev.xfac=(float)(lcddev.width-40)/(pos_temp[1][0]-pos_temp[0][0]);//得到xfac		 
					tp_dev.xoff=(lcddev.width-tp_dev.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;//得到xoff
						  
					tp_dev.yfac=(float)(lcddev.height-40)/(pos_temp[2][1]-pos_temp[0][1]);//得到yfac
					tp_dev.yoff=(lcddev.height-tp_dev.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;//得到yoff  
					if(fabs(tp_dev.xfac)>2||fabs(tp_dev.yfac)>2)//触屏和预设的相反了.
					{
						cnt=0;
						TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
						TP_Drow_Touch_Point(20,20,RED);								//画点1
						tp_dev.touchtype=!tp_dev.touchtype;//修改触屏类型.
						if(tp_dev.touchtype)//X,Y方向与屏幕相反
						{
							TP_CMD_RDX =0X90;
							TP_CMD_RDY=0XD0;	 
						}else				   //X,Y方向与屏幕相同
						{
							TP_CMD_RDX=0XD0;
							TP_CMD_RDY=0X90;	 
						}			    
						continue;
					}		
					LCD_Clear(WHITE);//清屏
					LCD_brush_color=BLUE;
					LCD_board_color=WHITE;
					LCD_chinese_font=Chinese_2424;
					LCD_Display_Chinese(355,220,"屏幕校准完成");//校正完成
					my_delayms(1000);
					TP_Save_Adjdata();  
 					LCD_Clear(WHITE);//清屏   
					return;//校正完成				 
			}
		}
 	}
}	 


//判断触摸点是否在某一区域内
uint8_t TP_Area_Press_Judge(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey)
{
	if(tp_dev.x>sx && tp_dev.x<ex && tp_dev.y>sy && tp_dev.y<ey)
		return 1;
	else
		return 0;
}

//触摸屏初始化  		    
//返回值:	0,没有进行校准
//       	1,进行过校准
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
	
	TP_Read_XY(&tp_dev.x,&tp_dev.y);//第一次读取初始化
	if(TP_Get_Adjdata())
		return 0;//已校准
	else			 //未校准
	{ 										    
		LCD_Clear(WHITE);	//清屏
		TP_Adjust();  		//屏幕校准  
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
