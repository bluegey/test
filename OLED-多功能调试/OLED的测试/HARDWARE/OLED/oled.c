#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "delay.h"
//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
/*	.
	.
	.
	.
	.	*/
//[63]0 1 2 3 ... 127
/**********************************************
//IIC Start
**********************************************/
/**********************************************
//IIC Start
**********************************************/
uint8_t OLED_GRAM[64][128];

void IIC_Start()
{

    OLED_SCLK_Set() ;
    OLED_SDIN_Set();
    OLED_SDIN_Clr();
    OLED_SCLK_Clr();
}

/**********************************************
//IIC Stop
**********************************************/
void IIC_Stop()
{
    OLED_SCLK_Set() ;
//	OLED_SCLK_Clr();
    OLED_SDIN_Clr();
    OLED_SDIN_Set();

}

void IIC_Wait_Ack()
{

    //GPIOB->CRH &= 0XFFF0FFFF;	//设置PB12为上拉输入模式
    //GPIOB->CRH |= 0x00080000;
//	OLED_SDA = 1;
//	delay_us(1);
    //OLED_SCL = 1;
    //delay_us(50000);
    /*	while(1)
    	{
    		if(!OLED_SDA)				//判断是否接收到OLED 应答信号
    		{
    			//GPIOB->CRH &= 0XFFF0FFFF;	//设置PB12为通用推免输出模式
    			//GPIOB->CRH |= 0x00030000;
    			return;
    		}
    	}
    */
    OLED_SCLK_Set() ;
    OLED_SCLK_Clr();
}
/**********************************************
// IIC Write byte
**********************************************/
	unsigned char Final_data=0;
void Write_IIC_Byte(unsigned char IIC_Byte)
{
    unsigned char i;
    unsigned char m,da;
    da=IIC_Byte;
    OLED_SCLK_Clr();
    for(i=0; i<8; i++)
    {
        m=da;
        //	OLED_SCLK_Clr();
        m=m&0x80;
        if(m==0x80)
        {
            OLED_SDIN_Set();
        }
        else OLED_SDIN_Clr();
        da=da<<1;
        OLED_SCLK_Set();
        OLED_SCLK_Clr();
    }
}
/**********************************************
// IIC Write Command
**********************************************/
void Write_IIC_Command(unsigned char IIC_Command)
{
    IIC_Start();
    Write_IIC_Byte(0x78);            //Slave address,SA0=0
    IIC_Wait_Ack();
    Write_IIC_Byte(0x00);			//write command
    IIC_Wait_Ack();
    Write_IIC_Byte(IIC_Command);
    IIC_Wait_Ack();
    IIC_Stop();
}
/**********************************************
// IIC Write Data
**********************************************/
void Write_IIC_Data(unsigned char IIC_Data)
{
    IIC_Start();
    Write_IIC_Byte(0x78);			//D/C#=0; R/W#=0
    IIC_Wait_Ack();
    Write_IIC_Byte(0x40);			//write data
    IIC_Wait_Ack();
    Write_IIC_Byte(IIC_Data);
    IIC_Wait_Ack();
    IIC_Stop();
}
void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
    if(cmd)
    {

        Write_IIC_Data(dat);

    }
    else {
        Write_IIC_Command(dat);

    }
}
/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
    unsigned char m,n;
    for(m=0; m<8; m++)
    {
        OLED_WR_Byte(0xb0+m,0);		//page0-page1
        OLED_WR_Byte(0x00,0);		//low column start address
        OLED_WR_Byte(0x10,0);		//high column start address
        for(n=0; n<128; n++)
        {
            OLED_WR_Byte(fill_Data,1);
        }
    }
}


/***********************Delay****************************************/
void Delay_50ms(unsigned int Del_50ms)
{
    unsigned int m;
    for(; Del_50ms>0; Del_50ms--)
        for(m=6245; m>0; m--);
}

void Delay_1ms(unsigned int Del_1ms)
{
    unsigned char j;
    while(Del_1ms--)
    {
        for(j=0; j<123; j++);
    }
}

//坐标设置
void OLED_Set_Pos(unsigned char x, unsigned char y)
{   OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f),OLED_CMD);
}
//开启OLED显示  OLED_GRAM
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void OLED_Clear(void)
{
    u8 i,n;
    for(i=0; i<8; i++)
    {
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
        OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
        OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址
        for(n=0; n<128; n++)OLED_WR_Byte(0x00,OLED_DATA);
    } //更新显示
}
//更新显存到OLED	
void OLED_Refresh(void)
{
	uint8_t i,n;
	for(i=0;i<16;i++)
	{
		OLED_WR_Byte(0xb0+i,OLED_CMD); //设置行起始地址
		OLED_WR_Byte(0x00,OLED_CMD);   //设置低列起始地址
		OLED_WR_Byte(0x10,OLED_CMD);   //设置高列起始地址
		IIC_Start();
		OLED_WR_Byte(0x78,OLED_CMD);
		IIC_Wait_Ack();
		OLED_WR_Byte(0x40,OLED_CMD);
		IIC_Wait_Ack();
		for(n=0;n<80;n++)
		{
			OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
			IIC_Wait_Ack();
		}
		IIC_Stop();
  }
}
void OLED_On(void)
{
    u8 i,n;
    for(i=0; i<8; i++)
    {
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7） OLED_WR_Byte
        OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
        OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址
        for(n=0; n<128; n++)OLED_WR_Byte(1,OLED_DATA);
    } //更新显示
}
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12
void Display(u8 x,u8 y,u8 chr)
{
  OLED_Set_Pos(x,y);
	for(int i=0;i<8;i++)
	{ OLED_WR_Byte(data[chr][i],OLED_DATA);}

}

void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size)
{
    unsigned char c=0,i=0;

    c=chr-' ';//得到偏移后的值
    if(x>Max_Column-1) {
        x=0;
        y=y+2;
    }
    if(Char_Size ==16)
    {
       
        for(i=0; i<8; i++)
			{
#if (Reverse_display==1)
       			OLED_Set_Pos(127-x-i,7-y);
			      Final_data=transform_data(F8X16[c*16+i]);
            OLED_WR_Byte(Final_data,OLED_DATA);
#elif (Reverse_display==0)
				OLED_Set_Pos(x+i,y);
				 OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
#endif
			}
       
        for(i=0; i<8; i++) 
			{
#if (Reverse_display==1)
				    OLED_Set_Pos(127-x-i,7-(y+1));
				    Final_data=transform_data(F8X16[c*16+i+8]);
            OLED_WR_Byte(Final_data,OLED_DATA);
#elif (Reverse_display==0)
				OLED_Set_Pos(x+i,y+1);
				 OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
#endif
			}
    }
    else {
       
        for(i=0; i<6; i++)
			{
#if (Reverse_display==1)
				Final_data=transform_data(F6x8[c][i]);
				   OLED_Set_Pos(127-x-i,7-y);		
            OLED_WR_Byte(Final_data,OLED_DATA);
#elif (Reverse_display==0)
				OLED_Set_Pos(x+i,y);
				 OLED_WR_Byte(F6x8[c][i],OLED_DATA);
#endif				
			}

    }
}
#if (Reverse_display==1)
unsigned char transform_data(unsigned char data)
{
    unsigned char tem=0;
	  for(int i=0;i<8;i++)
			{
        tem = (0x01 & (data >>i))<<(7-i)|tem;  
			}

   return tem;

}
#endif
//m^n函数
u32 oled_pow(u8 m,u8 n)
{
    u32 result=1;
    while(n--)result*=m;
    return result;
}
//显示2个数字
//x,y :起点坐标
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2)
{
    u8 t,temp;
    u8 enshow=0;
    for(t=0; t<len; t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+8*t,y,' ',size2);
                continue;
            } else enshow=1;

        }
        OLED_ShowChar(x+8*t,y,temp+'0',size2);
    }
}
//显示一个字符号串
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 Char_Size)
{
//    unsigned char j=0;
    while (*chr!='\0')
    {   OLED_ShowChar(x,y,*chr,Char_Size);
        x+=8;
        if(x>120) {
            x=0;
            y+=2;
        }
        chr++;
    }
}
void OLED_ShowStr(u8 x,u8 y,u8 *chr,u8 Char_Size)
{
//    unsigned char j=0;
    while (*chr!='\0')
    {   OLED_ShowChar(x,y,*chr,Char_Size);
        x+=1;
        if(x>120) {
            x=0;
            y+=2;
        }
        chr++;
    }
}
//显示汉字
void OLED_ShowCHinese(u8 x,u8 y,u8 no)
{
    u8 t; 
    for(t=0; t<16; t++)
    {
#if (Reverse_display==1)
			  Final_data=transform_data(Hzk[2*no][t]);
			  OLED_Set_Pos(127-x-t,7-y);
        OLED_WR_Byte(Final_data,OLED_DATA);
#elif (Reverse_display==0)
        OLED_Set_Pos(x+t,y);
			  OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
#endif
//        adder+=1;
    }
   
    for(t=0; t<16; t++)
    { 
#if (Reverse_display==1)
			 Final_data=transform_data(Hzk[2*no+1][t]);
			OLED_Set_Pos(127-x-t,7-(y+1));
        OLED_WR_Byte(Final_data,OLED_DATA);
#elif (Reverse_display==0)
        OLED_Set_Pos(x+t,y+1);
			  OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
#endif			
//        adder+=1;
    }
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{
    unsigned int j=0; //定义变量
    unsigned char x,y; //定义变量

    if(y1%8==0) y=y1/8;   //判断终止页是否为8的整数倍
    else y=y1/8+1;

    for(y=y0; y<y1; y++) //从起始页开始，画到终止页
    {
        OLED_Set_Pos(x0,y); //在页的起始列开始画
        for(x=x0; x<x1; x++) //画x1 - x0 列
        {
            OLED_WR_Byte(BMP[j++],OLED_DATA);	//画图片的点
            delay_ms(5);
        }

    }
}
/***********************************************************************************************/
//画点 
//x:0~127
//y:0~63
//t:1 填充 0,清空				   
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//超出范围了.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}
/*************************************************************************************************/
//初始化SSD1306   
void OLED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz GPIO_Pin_2
//    GPIO_Init(GPIOF, &GPIO_InitStructure);
//    GPIO_SetBits(GPIOF,GPIO_Pin_6|GPIO_Pin_7);
////	
//	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	  	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz GPIO_Pin_2
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA,GPIO_Pin_15);
	
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz GPIO_Pin_2
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB,GPIO_Pin_5); 
  
    delay_ms(200);
    OLED_WR_Byte(0xAE,OLED_CMD); //关闭显示  0xAE关闭显示命令  0xAF开启显示命令
		
    OLED_WR_Byte(0xD5,OLED_CMD); //设置时钟分频因子,震荡频率
    OLED_WR_Byte(0x80,OLED_CMD);   //[3:0],分频因子;[7:4],震荡频率
 
    OLED_WR_Byte(0xA8,OLED_CMD); //设置驱动路数
    OLED_WR_Byte(0X3F,OLED_CMD); //默认0X3F(1/64)
		
    OLED_WR_Byte(0xD3,OLED_CMD); //设置显示偏移
    OLED_WR_Byte(0X00,OLED_CMD); //默认为0
    OLED_WR_Byte(0x40,OLED_CMD); //设置显示开始行 [5:0],行数.

    OLED_WR_Byte(0x8D,OLED_CMD); //电荷泵设置
    OLED_WR_Byte(0x14,OLED_CMD); //bit2，开启/关闭
		
    OLED_WR_Byte(0x20,OLED_CMD); //设置内存地址模式
    OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
		
    OLED_WR_Byte(0xA1,OLED_CMD); //段重定义设置,bit0:0,0->0;1,0->127;0XA0

    OLED_WR_Byte(0xC8,OLED_CMD); //设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数0XC0
		
    OLED_WR_Byte(0xDA,OLED_CMD); //设置COM硬件引脚配置
    
		OLED_WR_Byte(0x12,OLED_CMD); //[5:4]配置

    OLED_WR_Byte(0x81,OLED_CMD); //对比度设置
    OLED_WR_Byte(0xEF,OLED_CMD); //1~255;默认0X7F (亮度设置,越大越亮)
    
		OLED_WR_Byte(0xD9,OLED_CMD); //设置预充电周期
    OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
    
		OLED_WR_Byte(0xDB,OLED_CMD); //设置VCOMH 电压倍率
    OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

    OLED_WR_Byte(0xA4,OLED_CMD); //全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
    OLED_WR_Byte(0xA6,OLED_CMD); //设置显示方式;bit0:1,反相显示;0,正常显示
    OLED_WR_Byte(0xAF,OLED_CMD); //开启显示
}
