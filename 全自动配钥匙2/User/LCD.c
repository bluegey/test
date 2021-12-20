#include "LCD.h"
#include "stdio.h"
#include "stdarg.h"
#include "mydelay.h"

//硬件相关的子函数
#define Bank1_LCD_RAM    	((u32)0x6C020000)    //Disp Data ADDR
#define Bank1_LCD_REG  		((u32)0x6C01FFFE)	   //Disp Reg ADDR

#define LCD_BackLight_ON()   GPIO_SetBits(GPIOG, GPIO_Pin_7)
#define LCD_BackLight_OFF()  GPIO_ResetBits(GPIOG, GPIO_Pin_7)

_lcd_dev lcddev;
uint16_t LCD_brush_color=BLACK;
uint16_t LCD_board_color=WHITE;
uint8_t	 LCD_string_font=ASCII_0816;
uint8_t	 LCD_chinese_font=Chinese_2424;


void LCD_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMWriteTimingInitStructure; 
  FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMReadWriteTimingInitStructure;  
   
	/* Enable the FSMC Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
	
	/* config lcd gpio clock base on FSMC */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOG|RCC_APB2Periph_AFIO, ENABLE);
    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	/* config tft rst gpio ->PB6*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* config tft BL gpio base on the PT4101 ->PG7*/	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ; 	 
	GPIO_Init(GPIOG, &GPIO_InitStructure);  		   
	
	/* config tft data lines base on FSMC
	* data lines,FSMC-D0~D15: PD 14 15 0 1,PE 7 8 9 10 11 12 13 14 15,PD 8 9 10
	*/	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 | 
																GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
																GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
																GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
    
	/* config tft control lines base on FSMC
	* PD4-FSMC_NOE  :LCD-RD
	* PD5-FSMC_NWE  :LCD-WR
	* PG12-FSMC_NE4 :LCD-CS
	* PD11-FSMC_A16 :LCD-DC
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
	GPIO_Init(GPIOG, &GPIO_InitStructure);  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);  
    
	/* tft control gpio init */	 
	GPIO_SetBits(GPIOD, GPIO_Pin_4);		 // RD = 1  
	GPIO_SetBits(GPIOD, GPIO_Pin_5);		 // WR = 1 
	GPIO_SetBits(GPIOD, GPIO_Pin_11);		 // RS
	GPIO_SetBits(GPIOG, GPIO_Pin_12);		 //	CS = 1

	LCD_BackLight_OFF(); 
	
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMReadWriteTimingInitStructure;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMWriteTimingInitStructure; 
	FSMC_NORSRAMStructInit(&FSMC_NORSRAMInitStructure);
  FSMC_NORSRAMDeInit(FSMC_Bank1_NORSRAM4);
	
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_AddressSetupTime = 0x01;	 //地址建立时间
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_AddressHoldTime = 0x00;	 //地址保持时间
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_DataSetupTime = 0x0F;		 //数据建立时间
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_CLKDivision = 0x00;
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_DataLatency = 0x00;
	FSMC_NORSRAMReadWriteTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	 // 一般使用模式A来控制LCD
	
	FSMC_NORSRAMWriteTimingInitStructure.FSMC_AddressSetupTime = 0x00;	 //地址建立时间
	FSMC_NORSRAMWriteTimingInitStructure.FSMC_AddressHoldTime = 0x00;	 //地址保持时间
	FSMC_NORSRAMWriteTimingInitStructure.FSMC_DataSetupTime = 0x03;		 //数据建立时间
	FSMC_NORSRAMWriteTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
	FSMC_NORSRAMWriteTimingInitStructure.FSMC_CLKDivision = 0x00;
	FSMC_NORSRAMWriteTimingInitStructure.FSMC_DataLatency = 0x00;
	FSMC_NORSRAMWriteTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	 // 一般使用模式A来控制LCD
	
	
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;//储存块1区号4
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;//
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;//储存器类型
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;//数据宽度
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;//扩展
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;//设置写使能
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMReadWriteTimingInitStructure;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMWriteTimingInitStructure; 
	
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
	/* Enable FSMC Bank1_SRAM Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);  
}


void LCD_Write_Cmd(uint16_t CMD)
{			
	*(__IO uint16_t *) (Bank1_LCD_REG) = CMD;
}
void LCD_Write_Data(uint16_t DATA)
{			
	*(__IO uint16_t *) (Bank1_LCD_RAM) = DATA;
}

uint16_t LCD_Read_Data(void)
{			
	vu16 DATA;
	DATA = *(__IO uint16_t *) (Bank1_LCD_RAM);
	return DATA;
}

void LCD_Write_Reg(uint16_t reg,uint16_t value)
{
	*(__IO uint16_t *) (Bank1_LCD_REG) = reg;
	*(__IO uint16_t *) (Bank1_LCD_RAM) = value;
}

uint16_t LCD_Read_Reg(uint16_t reg)
{
	vu16 value;
	*(__IO uint16_t *) (Bank1_LCD_REG) = reg;
	my_delayus(5);
	value = *(__IO uint16_t *) (Bank1_LCD_RAM);
	return value;
}

void LCD_Display_On(void)
{
	LCD_Write_Cmd(0x2900);
}

void LCD_Display_Off(void)
{
	LCD_Write_Cmd(0x2800);
}


//开始写GRAM
void LCD_WriteRAM_Prepare(void)
{
 	LCD_Write_Cmd(lcddev.wramcmd);	  
}

//设置光标位置
//Xpos:横坐标
//Ypos:纵坐标
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{	 
	LCD_Write_Cmd(lcddev.setxcmd);LCD_Write_Data(Xpos>>8); 		
	LCD_Write_Cmd(lcddev.setxcmd+1);LCD_Write_Data(Xpos&0XFF);			 
	LCD_Write_Cmd(lcddev.setycmd);LCD_Write_Data(Ypos>>8);  		
	LCD_Write_Cmd(lcddev.setycmd+1);LCD_Write_Data(Ypos&0XFF);			
} 		 


//画点
//x,y:坐标
//POINT_COLOR:此点的颜色
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	LCD_SetCursor(x,y);		//设置光标位置 
	LCD_WriteRAM_Prepare();	//开始写入GRAM
	*(__IO uint16_t *) (Bank1_LCD_RAM) = LCD_brush_color;
}

void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Write_Cmd(lcddev.setxcmd);LCD_Write_Data(x>>8); 		
	LCD_Write_Cmd(lcddev.setxcmd+1);LCD_Write_Data(x&0XFF);			 
	LCD_Write_Cmd(lcddev.setycmd);LCD_Write_Data(y>>8);  		
	LCD_Write_Cmd(lcddev.setycmd+1);LCD_Write_Data(y&0XFF);
	LCD_WriteRAM_Prepare();	//开始写入GRAM
	*(__IO uint16_t *) (Bank1_LCD_RAM) = color;
}

//设置窗口,并自动设置画点坐标到窗口左上角(sx,sy).
//sx,sy:窗口起始坐标(左上角)
//width,height:窗口宽度和高度,必须大于0!!
//窗体大小:width*height. 
void LCD_Set_Window(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey)
{    
	ex-=1;
	ey-=1;
	LCD_Write_Cmd(lcddev.setxcmd);LCD_Write_Data(sx>>8);  
	LCD_Write_Cmd(lcddev.setxcmd+1);LCD_Write_Data(sx&0XFF);	  
	LCD_Write_Cmd(lcddev.setxcmd+2);LCD_Write_Data(ex>>8);   
	LCD_Write_Cmd(lcddev.setxcmd+3);LCD_Write_Data(ex&0XFF);   
	LCD_Write_Cmd(lcddev.setycmd);LCD_Write_Data(sy>>8);   
	LCD_Write_Cmd(lcddev.setycmd+1);LCD_Write_Data(sy&0XFF);  
	LCD_Write_Cmd(lcddev.setycmd+2);LCD_Write_Data(ey>>8);   
	LCD_Write_Cmd(lcddev.setycmd+3);LCD_Write_Data(ey&0XFF);  
}

//清屏函数
//color:要清屏的填充色
void LCD_Clear(uint16_t color)
{
	u32 index=0;      
	u32 totalpoint=lcddev.width;
	totalpoint*=lcddev.height; 			//得到总点数
	LCD_Set_Window(0,0,lcddev.width,lcddev.height);	//设置光标位置 
	LCD_WriteRAM_Prepare();     		//开始写入GRAM	 	  
	for(index=0;index<totalpoint;index++)
	{
		*(__IO uint16_t *) (Bank1_LCD_RAM) = color;
	}
}  

//在指定区域内填充单个颜色
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)   
//color:要填充的颜色
void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color)
{          
	u32 index=0;
	u32 totalpoint=lcddev.width;
	totalpoint*=lcddev.height;	
	LCD_Set_Window(sx,sy,ex,ey);	
	LCD_WriteRAM_Prepare();     		//开始写入GRAM	 	  
	for(index=0;index<totalpoint;index++)
	{
		*(__IO uint16_t *) (Bank1_LCD_RAM) = color;
	}	 
} 

//画线
//x1,y1:起点坐标
//x2,y2:终点坐标  
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		LCD_DrawPoint(uRow,uCol);//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}

//画矩形	  
//(x1,y1),(x2,y2):矩形的对角坐标
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}

//在指定位置画一个指定大小的圆
//(x,y):中心点
//r    :半径
void LCD_Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //判断下个点位置的标志
	while(a<=b)
	{
		LCD_DrawPoint(x0+a,y0-b);             //5
 		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-a,y0+b);             //1       
 		LCD_DrawPoint(x0-b,y0+a);             
		LCD_DrawPoint(x0-a,y0-b);             //2             
  	LCD_DrawPoint(x0-b,y0-a);             //7     	         
		a++;
		//使用Bresenham算法画圆     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
} 


 
void LCD_Display_Format(uint16_t x,uint16_t y,const char *fmt,...)
{         	
	va_list ap;
	char string[64]={0};
	va_start(ap,fmt);
	vsnprintf(string,64,fmt,ap);
	va_end(ap);
	LCD_Display_String(x,y,(uint8_t*)string);
} 


void LCD_Display_Font(uint16_t x,uint16_t y,uint8_t type,uint8_t* GBCode)
{
	uint8_t e,i,j,ed;
	uint8_t ncols,nrows,nbytes;
	uint8_t Buffer[128]/*={

 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x00,0x00,0xC0,0x00,0x00,0xC0,0x00,0x1F,0xFF,0xFC,0x1F,0xFF,0xFC,0x18,0x00,0x0C,
 0x18,0x00,0x0C,0x18,0x00,0x0C,0x18,0xFF,0x8C,0x18,0xFF,0x8C,0x18,0xC1,0x8C,0x18,0xC1,0x8C,0x18,0xC1,0x8C,0x18,0xC1,0x8C,
 0x18,0xFF,0x8C,0x18,0xFF,0x8C,0x18,0x00,0x0C,0x18,0x00,0x0C,0x18,0x00,0x0C,0x18,0x00,0x7C,0x18,0x00,0x38,0x00,0x00,0x00}*/;
	unsigned long address;

	address=GetChineseAddress(type,GBCode,&nbytes,&ncols,&nrows);
	GT30L32S4W_Read(address,nbytes,Buffer);
	
	LCD_Set_Window(x, y, x+ncols, y+nrows);
  LCD_WriteRAM_Prepare();

	for(i=0;i<nbytes;i++)
	{ 	
//		Buffer[i]=0x12;
		ed=Buffer[i];		
		e=8;
		if(ncols%8)
		{
			if(ncols/8)
			{
				if(i%2)
					e=ncols%8;
			}
			else
				e=ncols%8;
				
		}	     
		for(j=0;j<e;j++)
		{
			if((ed<<j)&0x80)
				LCD_Write_Data(LCD_brush_color);  //textcolor
			else
				LCD_Write_Data(LCD_board_color);  //textcolor
		}
	}
}

void LCD_Display_String(uint16_t x,uint16_t y,uint8_t* GBCode)
{
	uint8_t x_add;
	if(LCD_string_font==ASCII_0507)				x_add=5;
	else if(LCD_string_font==ASCII_0708)		x_add=7;
	else if(LCD_string_font==ASCII_0612)		x_add=6;
	else if(LCD_string_font==ASCII_0816)		x_add=8;
	else if(LCD_string_font==ASCII_1224)		x_add=12;
	else if(LCD_string_font==ASCII_1632)		x_add=16;
	
	while(*GBCode!=0)
	{
		LCD_Display_Font(x,y,LCD_string_font,GBCode);
		GBCode++;
		x+=x_add;
	}
}

void LCD_Display_Chinese(uint16_t x,uint16_t y,uint8_t* GBCode)
{
	uint8_t x_add;
	if(LCD_chinese_font==Chinese_1112)				x_add=11;
	else if(LCD_chinese_font==Chinese_1516)		x_add=15;
	else if(LCD_chinese_font==Chinese_2424)		x_add=24;
	else if(LCD_chinese_font==Chinese_3232)		x_add=32;
	
	while(*GBCode!=0)
	{
		LCD_Display_Font(x,y,LCD_chinese_font,GBCode);
		GBCode+=2;
		x+=x_add;
	}
}


void LCD_Display_Message(uint16_t x,uint16_t y,uint8_t* GBCode)
{
	uint8_t x_add1,x_add2;
	if(LCD_chinese_font==Chinese_1112)				x_add1=11;
	else if(LCD_chinese_font==Chinese_1516)		x_add1=15;
	else if(LCD_chinese_font==Chinese_2424)		x_add1=24;
	else if(LCD_chinese_font==Chinese_3232)		x_add1=32;
	
	if(LCD_string_font==ASCII_0507)					x_add2=5;
	else if(LCD_string_font==ASCII_0708)		x_add2=7;
	else if(LCD_string_font==ASCII_0612)		x_add2=6;
	else if(LCD_string_font==ASCII_0816)		x_add2=8;
	else if(LCD_string_font==ASCII_1224)		x_add2=12;
	else if(LCD_string_font==ASCII_1632)		x_add2=16;
	
	while(*GBCode!=0)
	{
		if(*GBCode >= 0xA1)//当前显示的是汉字
		{
			LCD_Display_Font(x,y,LCD_chinese_font,GBCode);
			GBCode+=2;
			x+=x_add1;
		}
		else if((*GBCode >= 0x20) && (*GBCode <= 0x7E))//当前显示的是汉字
		{
			LCD_Display_Font(x,y,LCD_string_font,GBCode);
			GBCode+=1;
			x+=x_add2;
		}
		else//未定义字符
			break;
	}
}


//设置LCD显示方向
//dir:0,竖屏；1,横屏
void LCD_Display_Dir(uint8_t dir)
{
	if(dir==0)			//竖屏
	{
		lcddev.dir=0;	//竖屏
		lcddev.wramcmd=0X2C00;
		lcddev.setxcmd=0X2A00;
		lcddev.setycmd=0X2B00; 
		lcddev.width=480;
		lcddev.height=854;
	}
	else 				//横屏
	{	  				
		lcddev.dir=1;	//横屏
		lcddev.wramcmd=0X2C00;
		lcddev.setxcmd=0X2A00;
		lcddev.setycmd=0X2B00; 
		lcddev.width=854;
		lcddev.height=480;
	} 
	LCD_Scan_Dir(DFT_SCAN_DIR);	//默认扫描方向
}	 

//设置LCD的自动扫描方向
void LCD_Scan_Dir(uint8_t dir)
{
	uint16_t regval=0;
	uint16_t dirreg=0;
	uint16_t temp;  
	
	switch(dir)
	{
		case L2R_U2D://从左到右,从上到下
			regval|=(0<<7)|(0<<6)|(0<<5); 
			break;
		case L2R_D2U://从左到右,从下到上
			regval|=(1<<7)|(0<<6)|(0<<5); 
			break;
		case R2L_U2D://从右到左,从上到下
			regval|=(0<<7)|(1<<6)|(0<<5); 
			break;
		case R2L_D2U://从右到左,从下到上
			regval|=(1<<7)|(1<<6)|(0<<5); 
			break;	 
		case U2D_L2R://从上到下,从左到右
			regval|=(0<<7)|(0<<6)|(1<<5); 
			break;
		case U2D_R2L://从上到下,从右到左
			regval|=(0<<7)|(1<<6)|(1<<5); 
			break;
		case D2U_L2R://从下到上,从左到右
			regval|=(1<<7)|(0<<6)|(1<<5); 
			break;
		case D2U_R2L://从下到上,从右到左
			regval|=(1<<7)|(1<<6)|(1<<5); 
			break;	 
	}
	dirreg=0X3600;
 		   
	LCD_Write_Reg(dirreg,regval);

	if(regval&0X20)
	{
		if(lcddev.width<lcddev.height)//交换X,Y
		{
			temp=lcddev.width;
			lcddev.width=lcddev.height;
			lcddev.height=temp;
		}
	}
	else  
	{
		if(lcddev.width>lcddev.height)//交换X,Y
		{
			temp=lcddev.width;
			lcddev.width=lcddev.height;
			lcddev.height=temp;
		}
	}  

	LCD_Write_Cmd(lcddev.setxcmd);LCD_Write_Data(0); 
	LCD_Write_Cmd(lcddev.setxcmd+1);LCD_Write_Data(0); 
	LCD_Write_Cmd(lcddev.setxcmd+2);LCD_Write_Data((lcddev.width-1)>>8); 
	LCD_Write_Cmd(lcddev.setxcmd+3);LCD_Write_Data((lcddev.width-1)&0XFF); 
	LCD_Write_Cmd(lcddev.setycmd);LCD_Write_Data(0); 
	LCD_Write_Cmd(lcddev.setycmd+1);LCD_Write_Data(0); 
	LCD_Write_Cmd(lcddev.setycmd+2);LCD_Write_Data((lcddev.height-1)>>8); 
	LCD_Write_Cmd(lcddev.setycmd+3);LCD_Write_Data((lcddev.height-1)&0XFF);	
}     

void LCD_Init(void)
{
	uint16_t id=0;
	SPI_FLASH_Init();
	LCD_IO_Init();
	my_delayms(10);
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	my_delayms(10);					   
	GPIO_SetBits(GPIOA, GPIO_Pin_3);
	my_delayms(10);
	
	id=LCD_Read_Reg(0x0000);	//读ID（9320/9325/9328/4531/4535等IC）   
	if(id<0XFF||id==0XFFFF||id==0X9300)//读到ID不正确,新增lcddev.id==0X9300判断，因为9341在未被复位的情况下会被读成9300
	{	
		//尝试9341 ID的读取		
		LCD_Write_Cmd(0XD3);		//读取LCD型号		   
		id=LCD_Read_Data();	//dummy read 	
		id=LCD_Read_Data();	//读到0X00
		id=LCD_Read_Data();   	//读取93								   
		id<<=8;
		id|=LCD_Read_Data();  	//读取41 	   			   
		if(id!=0X9341)		//非9341,尝试是不是6804
		{	
			LCD_Write_Cmd(0XBF);				   
			id=LCD_Read_Data(); 	//dummy read 	 
			id=LCD_Read_Data();   	//读回0X01			   
			id=LCD_Read_Data(); 	//读回0XD0 			  	
			id=LCD_Read_Data();	//这里读回0X68 
			id<<=8;
			id|=LCD_Read_Data();	//这里读回0X04	  
			if(id!=0X6804)		//也不是6804,尝试看看是不是NT35310
			{ 
				LCD_Write_Cmd(0XD4);				   
				id=LCD_Read_Data();//dummy read  
				id=LCD_Read_Data();//读回0X01	 
				id=LCD_Read_Data();//读回0X53	
				id<<=8;	 
				id|=LCD_Read_Data();	//这里读回0X10	 
				if(id!=0X5310)		//也不是NT35310,尝试看看是不是NT35510
				{
					LCD_Write_Cmd(0XDA00);	
					id=LCD_Read_Data();		//读回0X00	 
					LCD_Write_Cmd(0XDB00);	
					id=LCD_Read_Data();		//读回0X80
					id<<=8;	
					LCD_Write_Cmd(0XDC00);	
					id|=LCD_Read_Data();		//读回0X00		
					if(id==0x8000)
						id=0x5510;//NT35510读回的ID是8000H,为方便区分,我们强制设置为5510
					if(id!=0X5510)			//也不是NT5510,尝试看看是不是SSD1963
					{
						LCD_Write_Cmd(0XA1);
						id=LCD_Read_Data();
						id=LCD_Read_Data();	//读回0X57
						id<<=8;	 
						id|=LCD_Read_Data();	//读回0X61	
						if(id==0X5761)
							id=0X1963;//SSD1963读回的ID是5761H,为方便区分,我们强制设置为1963
					}
				}
			}
		}  	
	} 
	lcddev.id=id;
	//Enable Page1                                 
	LCD_Write_Cmd(0xF000);LCD_Write_Data(0x55);              
	LCD_Write_Cmd(0xF001);LCD_Write_Data(0xaa);              
	LCD_Write_Cmd(0xF002);LCD_Write_Data(0x52);              
	LCD_Write_Cmd(0xF003);LCD_Write_Data(0x08);              
	LCD_Write_Cmd(0xF004);LCD_Write_Data(0x01);              

	// AVDD: manual,                                
	LCD_Write_Cmd(0xB600);LCD_Write_Data(0x34);               
	LCD_Write_Cmd(0xB601);LCD_Write_Data(0x34);               
	LCD_Write_Cmd(0xB602);LCD_Write_Data(0x34);               
	LCD_Write_Cmd(0xB000);LCD_Write_Data(0x09);               
	LCD_Write_Cmd(0xB001);LCD_Write_Data(0x09);               
	LCD_Write_Cmd(0xB002);LCD_Write_Data(0x09);               

	// AVEE: manual,       ‐‐6V                   

	LCD_Write_Cmd(0xB700);LCD_Write_Data(0x24);               
	LCD_Write_Cmd(0xB701);LCD_Write_Data(0x24);               
	LCD_Write_Cmd(0xB702);LCD_Write_Data(0x24);               
	LCD_Write_Cmd(0xB100);LCD_Write_Data(0x09);               
	LCD_Write_Cmd(0xB101);LCD_Write_Data(0x09);               
	LCD_Write_Cmd(0xB102);LCD_Write_Data(0x90);               

	//Power Control for VCL                         
	LCD_Write_Cmd(0xB800);LCD_Write_Data(0x34);               
	LCD_Write_Cmd(0xB200);LCD_Write_Data(0x00);               

	// VGH: Clamp Enable,                           
	LCD_Write_Cmd(0xB900);LCD_Write_Data(0x24);               
	LCD_Write_Cmd(0xB901);LCD_Write_Data(0x24);               
	LCD_Write_Cmd(0xB902);LCD_Write_Data(0x24);               
	LCD_Write_Cmd(0xB300);LCD_Write_Data(0x05);               
	LCD_Write_Cmd(0xB301);LCD_Write_Data(0x05);               
	LCD_Write_Cmd(0xB302);LCD_Write_Data(0x05);               

	LCD_Write_Cmd(0xBF00);LCD_Write_Data(0x01);               
                                                                
	// VGL(LVGL)                                       
	LCD_Write_Cmd(0xBA00);LCD_Write_Data(0x34 );                
	LCD_Write_Cmd(0xBA01);LCD_Write_Data(0x34 );                
	LCD_Write_Cmd(0xBA02);LCD_Write_Data(0x34 );                

	// VGL_REG(VGLO)                                   
	LCD_Write_Cmd(0xB500);LCD_Write_Data(0x0B );                
	LCD_Write_Cmd(0xB501);LCD_Write_Data(0x0B );                
	LCD_Write_Cmd(0xB502);LCD_Write_Data(0x0B );                

	// VGMP/VGSP                                       
	LCD_Write_Cmd(0xBC00);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xBC01);LCD_Write_Data(0xA3 );                
	LCD_Write_Cmd(0xBC02);LCD_Write_Data(0x00 );                

	//VGMN/VGSN                                        
	LCD_Write_Cmd(0xBD00);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xBD01);LCD_Write_Data(0xA3 );                
	LCD_Write_Cmd(0xBD02);LCD_Write_Data(0x00 );                

	// VCOM= ‐‐0.1                                   

	LCD_Write_Cmd(0xBE00);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xBE01);LCD_Write_Data(0x50 );                

	//R+                                               
	LCD_Write_Cmd(0xD100);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD101);LCD_Write_Data(0x37 );                
	LCD_Write_Cmd(0xD102);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD103);LCD_Write_Data(0x52 );                
	LCD_Write_Cmd(0xD104);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD105);LCD_Write_Data(0x7B );                
	LCD_Write_Cmd(0xD106);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD107);LCD_Write_Data(0x99 );                
	LCD_Write_Cmd(0xD108);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD109);LCD_Write_Data(0xB1 );                
	LCD_Write_Cmd(0xD10A);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD10B);LCD_Write_Data(0xD2 );                
	LCD_Write_Cmd(0xD10C);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD10D);LCD_Write_Data(0xF6 );                
	LCD_Write_Cmd(0xD10E);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD10F);LCD_Write_Data(0x27 );                
	LCD_Write_Cmd(0xD110);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD111);LCD_Write_Data(0x4E );                
	LCD_Write_Cmd(0xD112);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD113);LCD_Write_Data(0x8C );                
	LCD_Write_Cmd(0xD114);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD115);LCD_Write_Data(0xBE );                
	LCD_Write_Cmd(0xD116);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD117);LCD_Write_Data(0x0B );                
	LCD_Write_Cmd(0xD118);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD119);LCD_Write_Data(0x48 );                
	LCD_Write_Cmd(0xD11A);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD11B);LCD_Write_Data(0x4A );                
	LCD_Write_Cmd(0xD11C);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD11D);LCD_Write_Data(0x7E );                
	LCD_Write_Cmd(0xD11E);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD11F);LCD_Write_Data(0xBC );                
	LCD_Write_Cmd(0xD120);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD121);LCD_Write_Data(0xE1 );                
	LCD_Write_Cmd(0xD122);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD123);LCD_Write_Data(0x10 );                
	LCD_Write_Cmd(0xD124);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD125);LCD_Write_Data(0x31 );                
	LCD_Write_Cmd(0xD126);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD127);LCD_Write_Data(0x5A );                
	LCD_Write_Cmd(0xD128);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD129);LCD_Write_Data(0x73 );                
	LCD_Write_Cmd(0xD12A);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD12B);LCD_Write_Data(0x94 );                
	LCD_Write_Cmd(0xD12C);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD12D);LCD_Write_Data(0x9F );                
	LCD_Write_Cmd(0xD12E);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD12F);LCD_Write_Data(0xB3 );                
	LCD_Write_Cmd(0xD130);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD131);LCD_Write_Data(0xB9 );                
	LCD_Write_Cmd(0xD132);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD133);LCD_Write_Data(0xC1 );                

	//G+                                               
	LCD_Write_Cmd(0xD200);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD201);LCD_Write_Data(0x37 );                
	LCD_Write_Cmd(0xD202);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD203);LCD_Write_Data(0x52 );                
	LCD_Write_Cmd(0xD204);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD205);LCD_Write_Data(0x7B );                
	LCD_Write_Cmd(0xD206);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD207);LCD_Write_Data(0x99 );                
	LCD_Write_Cmd(0xD208);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD209);LCD_Write_Data(0xB1 );                
	LCD_Write_Cmd(0xD20A);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD20B);LCD_Write_Data(0xD2 );                
	LCD_Write_Cmd(0xD20C);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD20D);LCD_Write_Data(0xF6 );                
	LCD_Write_Cmd(0xD20E);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD20F);LCD_Write_Data(0x27 );                
	LCD_Write_Cmd(0xD210);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD211);LCD_Write_Data(0x4E );                
	LCD_Write_Cmd(0xD212);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD213);LCD_Write_Data(0x8C );                
	LCD_Write_Cmd(0xD214);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD215);LCD_Write_Data(0xBE );                
	LCD_Write_Cmd(0xD216);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD217);LCD_Write_Data(0x0B );                
	LCD_Write_Cmd(0xD218);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD219);LCD_Write_Data(0x48 );                
	LCD_Write_Cmd(0xD21A);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD21B);LCD_Write_Data(0x4A );                
	LCD_Write_Cmd(0xD21C);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD21D);LCD_Write_Data(0x7E );                
	LCD_Write_Cmd(0xD21E);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD21F);LCD_Write_Data(0xBC );                
	LCD_Write_Cmd(0xD220);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD221);LCD_Write_Data(0xE1 );                
	LCD_Write_Cmd(0xD222);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD223);LCD_Write_Data(0x10 );                
	LCD_Write_Cmd(0xD224);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD225);LCD_Write_Data(0x31 );                
	LCD_Write_Cmd(0xD226);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD227);LCD_Write_Data(0x5A );                
	LCD_Write_Cmd(0xD228);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD229);LCD_Write_Data(0x73 );                
	LCD_Write_Cmd(0xD22A);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD22B);LCD_Write_Data(0x94 );                
	LCD_Write_Cmd(0xD22C);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD22D);LCD_Write_Data(0x9F );                
	LCD_Write_Cmd(0xD22E);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD22F);LCD_Write_Data(0xB3 );                
	LCD_Write_Cmd(0xD230);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD231);LCD_Write_Data(0xB9 );                
	LCD_Write_Cmd(0xD232);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD233);LCD_Write_Data(0xC1 );                

	//B+                                               
	LCD_Write_Cmd(0xD300);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD301);LCD_Write_Data(0x37 );                
	LCD_Write_Cmd(0xD302);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD303);LCD_Write_Data(0x52 );                
	LCD_Write_Cmd(0xD304);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD305);LCD_Write_Data(0x7B );                
	LCD_Write_Cmd(0xD306);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD307);LCD_Write_Data(0x99 );                
	LCD_Write_Cmd(0xD308);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD309);LCD_Write_Data(0xB1 );                
	LCD_Write_Cmd(0xD30A);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD30B);LCD_Write_Data(0xD2 );                
	LCD_Write_Cmd(0xD30C);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD30D);LCD_Write_Data(0xF6 );                
	LCD_Write_Cmd(0xD30E);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD30F);LCD_Write_Data(0x27 );                
	LCD_Write_Cmd(0xD310);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD311);LCD_Write_Data(0x4E );                
	LCD_Write_Cmd(0xD312);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD313);LCD_Write_Data(0x8C );                
	LCD_Write_Cmd(0xD314);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD315);LCD_Write_Data(0xBE );                
	LCD_Write_Cmd(0xD316);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD317);LCD_Write_Data(0x0B );                
	LCD_Write_Cmd(0xD318);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD319);LCD_Write_Data(0x48 );                
	LCD_Write_Cmd(0xD31A);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD31B);LCD_Write_Data(0x4A );                
	LCD_Write_Cmd(0xD31C);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD31D);LCD_Write_Data(0x7E );                
	LCD_Write_Cmd(0xD31E);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD31F);LCD_Write_Data(0xBC );                
	LCD_Write_Cmd(0xD320);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD321);LCD_Write_Data(0xE1 );                
	LCD_Write_Cmd(0xD322);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD323);LCD_Write_Data(0x10 );                
	LCD_Write_Cmd(0xD324);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD325);LCD_Write_Data(0x31 );                
	LCD_Write_Cmd(0xD326);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD327);LCD_Write_Data(0x5A );                
	LCD_Write_Cmd(0xD328);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD329);LCD_Write_Data(0x73 );                
	LCD_Write_Cmd(0xD32A);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD32B);LCD_Write_Data(0x94 );                
	LCD_Write_Cmd(0xD32C);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD32D);LCD_Write_Data(0x9F );                
	LCD_Write_Cmd(0xD32E);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD32F);LCD_Write_Data(0xB3 );                
	LCD_Write_Cmd(0xD330);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD331);LCD_Write_Data(0xB9 );                
	LCD_Write_Cmd(0xD332);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD333);LCD_Write_Data(0xC1 );                

	//R-                                               
	LCD_Write_Cmd(0xD400);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD401);LCD_Write_Data(0x37 );                
	LCD_Write_Cmd(0xD402);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD403);LCD_Write_Data(0x52 );                
	LCD_Write_Cmd(0xD404);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD405);LCD_Write_Data(0x7B );                
	LCD_Write_Cmd(0xD406);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD407);LCD_Write_Data(0x99 );                
	LCD_Write_Cmd(0xD408);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD409);LCD_Write_Data(0xB1 );                
	LCD_Write_Cmd(0xD40A);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD40B);LCD_Write_Data(0xD2 );                
	LCD_Write_Cmd(0xD40C);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD40D);LCD_Write_Data(0xF6 );                
	LCD_Write_Cmd(0xD40E);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD40F);LCD_Write_Data(0x27 );                
	LCD_Write_Cmd(0xD410);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD411);LCD_Write_Data(0x4E );                
	LCD_Write_Cmd(0xD412);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD413);LCD_Write_Data(0x8C );                
	LCD_Write_Cmd(0xD414);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD415);LCD_Write_Data(0xBE );                
	LCD_Write_Cmd(0xD416);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD417);LCD_Write_Data(0x0B );                
	LCD_Write_Cmd(0xD418);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD419);LCD_Write_Data(0x48 );                
	LCD_Write_Cmd(0xD41A);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD41B);LCD_Write_Data(0x4A );                
	LCD_Write_Cmd(0xD41C);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD41D);LCD_Write_Data(0x7E );                
	LCD_Write_Cmd(0xD41E);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD41F);LCD_Write_Data(0xBC );                
	LCD_Write_Cmd(0xD420);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD421);LCD_Write_Data(0xE1 );                
	LCD_Write_Cmd(0xD422);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD423);LCD_Write_Data(0x10 );                
	LCD_Write_Cmd(0xD424);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD425);LCD_Write_Data(0x31 );                
	LCD_Write_Cmd(0xD426);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD427);LCD_Write_Data(0x5A );                
	LCD_Write_Cmd(0xD428);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD429);LCD_Write_Data(0x73 );                
	LCD_Write_Cmd(0xD42A);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD42B);LCD_Write_Data(0x94 );                
	LCD_Write_Cmd(0xD42C);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD42D);LCD_Write_Data(0x9F );                
	LCD_Write_Cmd(0xD42E);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD42F);LCD_Write_Data(0xB3 );                
	LCD_Write_Cmd(0xD430);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD431);LCD_Write_Data(0xB9 );                
	LCD_Write_Cmd(0xD432);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD433);LCD_Write_Data(0xC1 );                

	//G-                                               
	LCD_Write_Cmd(0xD500);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD501);LCD_Write_Data(0x37 );                
	LCD_Write_Cmd(0xD502);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD503);LCD_Write_Data(0x52 );                
	LCD_Write_Cmd(0xD504);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD505);LCD_Write_Data(0x7B );                
	LCD_Write_Cmd(0xD506);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD507);LCD_Write_Data(0x99 );                
	LCD_Write_Cmd(0xD508);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD509);LCD_Write_Data(0xB1 );                
	LCD_Write_Cmd(0xD50A);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD50B);LCD_Write_Data(0xD2 );                
	LCD_Write_Cmd(0xD50C);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD50D);LCD_Write_Data(0xF6 );                
	LCD_Write_Cmd(0xD50E);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD50F);LCD_Write_Data(0x27 );                
	LCD_Write_Cmd(0xD510);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD511);LCD_Write_Data(0x4E );                
	LCD_Write_Cmd(0xD512);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD513);LCD_Write_Data(0x8C );                
	LCD_Write_Cmd(0xD514);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD515);LCD_Write_Data(0xBE );                
	LCD_Write_Cmd(0xD516);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD517);LCD_Write_Data(0x0B );                
	LCD_Write_Cmd(0xD518);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD519);LCD_Write_Data(0x48 );                
	LCD_Write_Cmd(0xD51A);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD51B);LCD_Write_Data(0x4A );                
	LCD_Write_Cmd(0xD51C);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD51D);LCD_Write_Data(0x7E );                
	LCD_Write_Cmd(0xD51E);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD51F);LCD_Write_Data(0xBC );                
	LCD_Write_Cmd(0xD520);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD521);LCD_Write_Data(0xE1 );                
	LCD_Write_Cmd(0xD522);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD523);LCD_Write_Data(0x10 );                
	LCD_Write_Cmd(0xD524);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD525);LCD_Write_Data(0x31 );                
	LCD_Write_Cmd(0xD526);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD527);LCD_Write_Data(0x5A );                
	LCD_Write_Cmd(0xD528);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD529);LCD_Write_Data(0x73 );                
	LCD_Write_Cmd(0xD52A);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD52B);LCD_Write_Data(0x94 );                
	LCD_Write_Cmd(0xD52C);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD52D);LCD_Write_Data(0x9F );                
	LCD_Write_Cmd(0xD52E);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD52F);LCD_Write_Data(0xB3 );                
	LCD_Write_Cmd(0xD530);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD531);LCD_Write_Data(0xB9 );                
	LCD_Write_Cmd(0xD532);LCD_Write_Data(0x03 );                
	LCD_Write_Cmd(0xD533);LCD_Write_Data(0xC1 );                

	//B-                                               
	LCD_Write_Cmd(0xD600);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD601);LCD_Write_Data(0x37 );                
	LCD_Write_Cmd(0xD602);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD603);LCD_Write_Data(0x52 );                
	LCD_Write_Cmd(0xD604);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD605);LCD_Write_Data(0x7B );                
	LCD_Write_Cmd(0xD606);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD607);LCD_Write_Data(0x99 );                
	LCD_Write_Cmd(0xD608);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD609);LCD_Write_Data(0xB1 );                
	LCD_Write_Cmd(0xD60A);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD60B);LCD_Write_Data(0xD2 );                
	LCD_Write_Cmd(0xD60C);LCD_Write_Data(0x00 );                
	LCD_Write_Cmd(0xD60D);LCD_Write_Data(0xF6 );                
	LCD_Write_Cmd(0xD60E);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD60F);LCD_Write_Data(0x27 );                
	LCD_Write_Cmd(0xD610);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD611);LCD_Write_Data(0x4E );                
	LCD_Write_Cmd(0xD612);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD613);LCD_Write_Data(0x8C );                
	LCD_Write_Cmd(0xD614);LCD_Write_Data(0x01 );                
	LCD_Write_Cmd(0xD615);LCD_Write_Data(0xBE );                
	LCD_Write_Cmd(0xD616);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD617);LCD_Write_Data(0x0B );                
	LCD_Write_Cmd(0xD618);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD619);LCD_Write_Data(0x48 );                
	LCD_Write_Cmd(0xD61A);LCD_Write_Data(0x02 );                
	LCD_Write_Cmd(0xD61B);LCD_Write_Data(0x4A );                
	LCD_Write_Cmd(0xD61C);LCD_Write_Data(0x02);                
	LCD_Write_Cmd(0xD61D);LCD_Write_Data(0x7E);                
	LCD_Write_Cmd(0xD61E);LCD_Write_Data(0x02);                
	LCD_Write_Cmd(0xD61F);LCD_Write_Data(0xBC);                
	LCD_Write_Cmd(0xD620);LCD_Write_Data(0x02);                
	LCD_Write_Cmd(0xD621);LCD_Write_Data(0xE1);                
	LCD_Write_Cmd(0xD622);LCD_Write_Data(0x03);                
	LCD_Write_Cmd(0xD623);LCD_Write_Data(0x10);                
	LCD_Write_Cmd(0xD624);LCD_Write_Data(0x03);                
	LCD_Write_Cmd(0xD625);LCD_Write_Data(0x31);                
	LCD_Write_Cmd(0xD626);LCD_Write_Data(0x03);                
	LCD_Write_Cmd(0xD627);LCD_Write_Data(0x5A);                
	LCD_Write_Cmd(0xD628);LCD_Write_Data(0x03);                
	LCD_Write_Cmd(0xD629);LCD_Write_Data(0x73);                
	LCD_Write_Cmd(0xD62A);LCD_Write_Data(0x03);                
	LCD_Write_Cmd(0xD62B);LCD_Write_Data(0x94);                
	LCD_Write_Cmd(0xD62C);LCD_Write_Data(0x03);                
	LCD_Write_Cmd(0xD62D);LCD_Write_Data(0x9F);                
	LCD_Write_Cmd(0xD62E);LCD_Write_Data(0x03);                
	LCD_Write_Cmd(0xD62F);LCD_Write_Data(0xB3);                
	LCD_Write_Cmd(0xD630);LCD_Write_Data(0x03);                
	LCD_Write_Cmd(0xD631);LCD_Write_Data(0xB9);                
	LCD_Write_Cmd(0xD632);LCD_Write_Data(0x03);                
	LCD_Write_Cmd(0xD633);LCD_Write_Data(0xC1);                

	//Enable Page0                                    
	LCD_Write_Cmd(0xF000);LCD_Write_Data(0x55);                
	LCD_Write_Cmd(0xF001);LCD_Write_Data(0xAA);                
	LCD_Write_Cmd(0xF002);LCD_Write_Data(0x52);                
	LCD_Write_Cmd(0xF003);LCD_Write_Data(0x08);                
	LCD_Write_Cmd(0xF004);LCD_Write_Data(0x00);                

	//RGB I/F Setting                                 
	LCD_Write_Cmd(0xB000);LCD_Write_Data(0x08);                
	LCD_Write_Cmd(0xB001);LCD_Write_Data(0x05);                
	LCD_Write_Cmd(0xB002);LCD_Write_Data(0x02);                
	LCD_Write_Cmd(0xB003);LCD_Write_Data(0x05);                
	LCD_Write_Cmd(0xB004);LCD_Write_Data(0x02);                

	// SDT:                                           
	LCD_Write_Cmd(0xB600);LCD_Write_Data(0x0A);                
	LCD_Write_Cmd(0xB500);LCD_Write_Data(0x6b);     // 顯示點陣 480x854      
	// Gate EQ:                                       
	LCD_Write_Cmd(0xB700);LCD_Write_Data(0x00);                
	LCD_Write_Cmd(0xB701);LCD_Write_Data(0x70);                

	// Source EQ                                      
	LCD_Write_Cmd(0xB800);LCD_Write_Data(0x01);                
	LCD_Write_Cmd(0xB801);LCD_Write_Data(0x05);                
	LCD_Write_Cmd(0xB802);LCD_Write_Data(0x05);                
	LCD_Write_Cmd(0xB803);LCD_Write_Data(0x05);                

	//Inversion: Column inversion (NVT);                
	LCD_Write_Cmd(0xBC00);LCD_Write_Data(0x02);                
	LCD_Write_Cmd(0xBC01);LCD_Write_Data(0x02);                
	LCD_Write_Cmd(0xBC02);LCD_Write_Data(0x02);                

	//BOE's Setting (default)                         
	LCD_Write_Cmd(0xCC00);LCD_Write_Data(0x03);                
	LCD_Write_Cmd(0xCC01);LCD_Write_Data(0x00);                
	LCD_Write_Cmd(0xCC02);LCD_Write_Data(0x00);                

	//Display Timing                                  
	LCD_Write_Cmd(0xBD00);LCD_Write_Data(0x01);                
	LCD_Write_Cmd(0xBD01);LCD_Write_Data(0x84);                
	LCD_Write_Cmd(0xBD02);LCD_Write_Data(0x07);                
	LCD_Write_Cmd(0xBD03);LCD_Write_Data(0x31);                
	LCD_Write_Cmd(0xBD04);LCD_Write_Data(0x00);                
	LCD_Write_Cmd(0xBA00);LCD_Write_Data(0x01);                
	LCD_Write_Cmd(0xFF00);LCD_Write_Data(0xAA);                
	LCD_Write_Cmd(0xFF01);LCD_Write_Data(0x55);                
	LCD_Write_Cmd(0xFF02);LCD_Write_Data(0x25);                
	LCD_Write_Cmd(0xFF03);LCD_Write_Data(0x01);                

	LCD_Write_Cmd(0x3A00);LCD_Write_Data(0x55);                

	LCD_Write_Cmd(0x1100);                                
	my_delayms(120);                                       
	LCD_Write_Cmd(0x2900 );
	
	LCD_Display_Dir(1);//横屏显示                               
	LCD_Clear(GREEN);
	LCD_BackLight_ON();
	
	LCD_board_color=GREEN;
	LCD_brush_color=BLACK;
	LCD_chinese_font=Chinese_3232;
	LCD_Display_Chinese(299,20,"全自动配钥匙系统");
	LCD_Display_Chinese(379,220,"启动中");
	LCD_chinese_font=Chinese_2424;
	LCD_Display_Chinese(355,450,"创新实验中心");
}
