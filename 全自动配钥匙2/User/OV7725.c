#include "OV7725.h"
#include "mydelay.h"

/* 寄存器宏定义 */
#define GAIN      0x00
#define BLUE      0x01
#define RED       0x02
#define GREEN     0x03
#define BAVG      0x05
#define GAVG      0x06
#define RAVG      0x07
#define AECH      0x08
#define COM2      0x09	//
#define PID       0x0A
#define VER       0x0B
#define COM3      0x0C	//
#define COM4      0x0D	//
#define COM5      0x0E	//
#define COM6      0x0F	//
#define AEC       0x10
#define CLKRC     0x11	//
#define COM7      0x12	//
#define COM8      0x13	//
#define COM9      0x14	//
#define COM10     0x15	//
#define REG16     0x16
#define HSTART    0x17	//
#define HSIZE     0x18	//
#define VSTRT     0x19	//
#define VSIZE     0x1A	//
#define PSHFT     0x1B
#define MIDH      0x1C
#define MIDL      0x1D
#define LAEC      0x1F
#define COM11     0x20	//
#define BDBase    0x22
#define BDMStep   0x23
#define AEW       0x24
#define AEB       0x25
#define VPT       0x26
#define REG28     0x28	//
#define HOutSize  0x29	//
#define EXHCH     0x2A	//
#define EXHCL     0x2B	//
#define VOutSize  0x2C	//
#define ADVFL     0x2D
#define ADVFH     0x2E
#define YAVE      0x2F
#define LumHTh    0x30
#define LumLTh    0x31
#define HREF      0x32	//
#define DM_LNL    0x33
#define DM_LNH    0x34
#define ADoff_B   0x35
#define ADoff_R   0x36
#define ADoff_Gb  0x37
#define ADoff_Gr  0x38
#define Off_B     0x39
#define Off_R     0x3A
#define Off_Gb    0x3B
#define Off_Gr    0x3C
#define COM12     0x3D
#define COM13     0x3E
#define COM14     0x3F
#define COM16     0x41
#define TGT_B     0x42
#define TGT_R     0x43
#define TGT_Gb    0x44
#define TGT_Gr    0x45
#define LC_CTR    0x46
#define LC_XC     0x47
#define LC_YC     0x48
#define LC_COEF   0x49
#define LC_RADI   0x4A
#define LC_COEFB  0x4B 
#define LC_COEFR  0x4C
#define FixGain   0x4D
#define AREF1     0x4F
#define AREF6     0x54
#define UFix      0x60
#define VFix      0x61
#define AWBb_blk  0x62
#define AWB_Ctrl0 0x63
#define DSP_Ctrl1 0x64
#define DSP_Ctrl2 0x65
#define DSP_Ctrl3 0x66
#define DSP_Ctrl4 0x67
#define AWB_bias  0x68
#define AWBCtrl1  0x69
#define AWBCtrl2  0x6A
#define AWBCtrl3  0x6B
#define AWBCtrl4  0x6C
#define AWBCtrl5  0x6D
#define AWBCtrl6  0x6E
#define AWBCtrl7  0x6F
#define AWBCtrl8  0x70
#define AWBCtrl9  0x71
#define AWBCtrl10 0x72
#define AWBCtrl11 0x73
#define AWBCtrl12 0x74
#define AWBCtrl13 0x75
#define AWBCtrl14 0x76
#define AWBCtrl15 0x77
#define AWBCtrl16 0x78
#define AWBCtrl17 0x79
#define AWBCtrl18 0x7A
#define AWBCtrl19 0x7B
#define AWBCtrl20 0x7C
#define AWBCtrl21 0x7D 
#define GAM1      0x7E
#define GAM2      0x7F
#define GAM3      0x80
#define GAM4      0x81
#define GAM5      0x82
#define GAM6      0x83
#define GAM7      0x84
#define GAM8      0x85
#define GAM9      0x86
#define GAM10     0x87
#define GAM11     0x88
#define GAM12     0x89
#define GAM13     0x8A
#define GAM14     0x8B
#define GAM15     0x8C
#define SLOP      0x8D
#define DNSTh     0x8E
#define EDGE0     0x8F
#define EDGE1     0x90
#define DNSOff    0x91
#define EDGE2     0x92
#define EDGE3     0x93
#define MTX1      0x94
#define MTX2      0x95
#define MTX3      0x96
#define MTX4      0x97
#define MTX5      0x98
#define MTX6      0x99
#define MTX_Ctrl  0x9A
#define BRIGHT    0x9B
#define CNST      0x9C
#define UVADJ0    0x9E
#define UVADJ1    0x9F
#define SCAL0     0xA0
#define SCAL1     0xA1
#define SCAL2     0xA2
#define SDE       0xA6
#define USAT      0xA7
#define VSAT      0xA8
#define HUECOS    0xA9
#define HUESIN    0xAA
#define SIGN      0xAB
#define DSPAuto   0xAC


/*宏定义*/
#define SCCB_SDA_READ()	(GPIOF->IDR & GPIO_Pin_8)

#define SCCB_SDA_H()	GPIOF->BSRR = GPIO_Pin_8
#define SCCB_SDA_L()	GPIOF->BRR = GPIO_Pin_8

#define SCCB_SCL_H()	GPIOF->BSRR = GPIO_Pin_9
#define SCCB_SCL_L()	GPIOF->BRR = GPIO_Pin_9


typedef struct Reg
{
	uint8_t Address;			       /*寄存器地址*/
	uint8_t Value;		           /*寄存器值*/
}Reg_Info;

/* 寄存器参数配置 */
Reg_Info Sensor_Config[] =
{
	{COM7,      0x00},//设置为YUV格式输出，640*480分辨率VGA
	{CLKRC,     0x10},
	{COM4,		  0x01},//PLL频率设置
	{EXHCH,			0x00},
	{EXHCL,			0x00},
	{DM_LNL,		0x00},
	{DM_LNH,		0x00},
	{ADVFL,			0x00},
	{ADVFH,			0x00},
	
	{COM5,			0x79},//关闭夜间模式
	
  {HSTART,    0x26},
	{HSIZE,     0xA0},
	{VSTRT,     0x07},
	{VSIZE,     0xF0},
	{HREF,      0x00},
	{HOutSize,  0xA0},
	{VOutSize,  0xF0},	 
	
	{COM3,      0x50},//设置YUV输出格式
	{COM10,     0x21}//PCLK不输出在消隐区
};

uint16_t chang_count=0;
uint16_t hang_count=0;

uint8_t camera_buff[2][10240];
uint8_t camera_buff_empty[2]={1,1};
uint8_t camera_buff_select=0;

//初始化IO接口
//PB0、PB1作为行场中断
//PB6作为TIM4IC1的输入捕捉触发DMA
void OV7725_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOF|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_DisableIRQ(EXTI0_IRQn);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource0);
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	TIM_DeInit(TIM2);
	TIM_TimeBaseInitStructure.TIM_Period = 32768;       //当定时器从0计数到999，即为1000次，为一个定时周期
	TIM_TimeBaseInitStructure.TIM_Prescaler = 64800;	    //设置预分频：不预分频，即为18MHz
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4 ;	//设置时钟分频系数：4分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICFilter=0;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV2;
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_DMACmd(TIM2,TIM_DMA_CC3,DISABLE);
	TIM_Cmd(TIM2, DISABLE);
	
	DMA_InitStructure.DMA_BufferSize=10240;
	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_M2M=DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr=(uint32_t)&camera_buff[0][0];
	DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode=DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)&GPIOF->IDR;
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority=DMA_Priority_High;
	DMA_Init(DMA1_Channel1,&DMA_InitStructure);
	
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Channel1,ENABLE);
}

//初始化SCCB接口
void SCCB_Init(void)
{											   
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOF,&GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOF,&GPIO_InitStructure);

}			 

//SCCB起始信号
//当时钟为高的时候,数据线的高到低,为SCCB起始信号
//在激活状态下,SDA和SCL均为低电平
void startSCCB(void)
{
	SCCB_SDA_H();     //数据线高电平	   
	SCCB_SCL_H();	    //在时钟线高的时候数据线由高至低
	my_delayus(2);  
	SCCB_SDA_L();
	my_delayus(2);	 
	SCCB_SCL_L();	    //数据线恢复低电平，单操作函数必要	  
}

//SCCB停止信号
//当时钟为高的时候,数据线的低到高,为SCCB停止信号
//空闲状况下,SDA,SCL均为高电平
void stopSCCB(void)
{
	SCCB_SDA_L();
	my_delayus(2);	 
	SCCB_SCL_H();	
	my_delayus(2); 
	SCCB_SDA_H();	
	my_delayus(2);
}


//noAck,用于连续读取中的最后一个结束周期
void noAck(void)
{
	my_delayus(2);
	SCCB_SDA_H();	
	SCCB_SCL_H();	
	my_delayus(2);
	SCCB_SCL_L();	
	my_delayus(2);
	SCCB_SDA_L();	
	my_delayus(2);
}
//SCCB,写入一个字节 
uint8_t SCCBwriteByte(uint8_t dat)
{
	uint8_t j,res;

	for(j=0;j<8;j++) //循环8次发送数据
	{
		if(dat&0x80)
			SCCB_SDA_H();	
		else 
			SCCB_SDA_L();
		dat<<=1;
		my_delayus(2);
		SCCB_SCL_H();	
		my_delayus(2);
		SCCB_SCL_L();		   
	}			 
	my_delayus(2);
	SCCB_SCL_H();	//接收第九位,以判断是否发送成功
	my_delayus(2);
	if(SCCB_SDA_READ())
		res=0;  //SDA=1发送失败，返回0}
	else 
		res=1;         //SDA=0发送成功，返回1
	SCCB_SCL_L();		  
	return res;  
}

//SCCB 读取一个字节
//在SCL的上升沿,数据锁存
uint8_t SCCBreadByte(void)
{
	uint8_t temp=0,j;    
	for(j=8;j>0;j--) //循环8次接收数据
	{		     	  
		my_delayus(2);
		SCCB_SCL_H();
		temp=temp<<1;
		if(SCCB_SDA_READ())
			temp++;   
		my_delayus(2);
		SCCB_SCL_L();
	}	
	return temp;
} 

////////////////////////////
//功能：写OV7670寄存器
//返回：1-成功	0-失败
uint8_t OV7725_Write_Reg(uint8_t regID,uint8_t regDat)
{
	startSCCB();
	if(0==SCCBwriteByte(0x42))
	{
		stopSCCB();
		return(0);
	}
	my_delayus(100);
	if(0==SCCBwriteByte(regID))
	{
		stopSCCB();
		return(0);
	}
	my_delayus(100);
	if(0==SCCBwriteByte(regDat))
	{
		stopSCCB();
		return(0);
	}
	stopSCCB();

	return(1);
}


////////////////////////////
//功能：读OV7670寄存器
//返回：1-成功	0-失败
uint8_t OV7725_Read_Reg(uint8_t regID,uint8_t *regDat)
{
	//通过写操作设置寄存器地址
	startSCCB();
	if(0==SCCBwriteByte(0x42))
	{
		stopSCCB();
		return(0);
	}
	my_delayus(100);
  	if(0==SCCBwriteByte(regID))
	{
		stopSCCB();
		return(0);
	}
	stopSCCB();
	
	my_delayus(100);
	//设置寄存器地址后，才是读
	startSCCB();
	if(0==SCCBwriteByte(0x43))
	{
		stopSCCB();
		return(0);
	}
	my_delayus(100);
	*regDat=SCCBreadByte();
	noAck();
	stopSCCB();
	return 1;
}


//初始化OV7670
//返回0:成功
//返回其他值:错误代码
uint8_t OV7725_init(void)
{
	uint8_t i,temp;
	
	SCCB_Init();        //初始化SCCB 的IO口	
	
	if(0==OV7725_Write_Reg(0x12,0x80))//软复位
		return 1 ; //Reset SCCB
	my_delayms(10);
	//读取产品型号
 	OV7725_Read_Reg(0x0b,&temp );   
	if(temp !=0x21)
		return 2;  
	OV7725_Read_Reg(0x0a,&temp); 
	if(temp !=0x77)
		return 2;
	//初始化序列	
	temp=sizeof(Sensor_Config)/sizeof(Sensor_Config[0]);
	for(i=0;i<temp;i++)
	{
		OV7725_Write_Reg(Sensor_Config[i].Address,Sensor_Config[i].Value);
		my_delayms(1);		
	}		 	
	
	OV7725_IO_Init();
	return 0x00; //ok
} 


//行中断
void EXTI0_IRQHandler(void)
{
	if ( EXTI_GetITStatus(EXTI_Line0) != RESET )
	{
		if(chang_count)
		{
			hang_count=0;
			camera_buff_select=0;
			camera_buff_empty[0]=1;
			camera_buff_empty[1]=1;
			
			TIM_DMACmd(TIM2,TIM_DMA_CC3,ENABLE);
			TIM_Cmd(TIM2, ENABLE);
			//复位预分频计数
			TIM2->CCER &= ~TIM_CCER_CC3E;
			TIM2->CCER |= TIM_CCER_CC3E;
			
			DMA_Cmd(DMA1_Channel1,DISABLE);
			DMA_SetCurrDataCounter(DMA1_Channel1,10240);//传输数据量
			DMA1_Channel1->CMAR=(uint32_t)&camera_buff[camera_buff_select][0];//数据传输
			DMA_Cmd(DMA1_Channel1,ENABLE);//开启DMA传输数据
		}
		chang_count++;			
		
		EXTI_ClearITPendingBit(EXTI_Line0);
	}		
}

//传输完成中断
void DMA1_Channel1_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC1)!= RESET)
	{
		camera_buff_empty[camera_buff_select]=0;
		hang_count+=16;
		camera_buff_select^=0x01;
		if(hang_count>=480)
		{
			TIM_DMACmd(TIM2,TIM_DMA_CC3,DISABLE);
			TIM_Cmd(TIM2, DISABLE);
			NVIC_DisableIRQ(EXTI0_IRQn);
//			chang_count=0;
		}
		else
		{
			if(!camera_buff_empty[camera_buff_select])
			{
				TIM_DMACmd(TIM2,TIM_DMA_CC3,DISABLE);
				NVIC_DisableIRQ(EXTI0_IRQn);
			}
			DMA_Cmd(DMA1_Channel1,DISABLE);
			DMA1_Channel1->CMAR=(uint32_t)&camera_buff[camera_buff_select][0];
			DMA_SetCurrDataCounter(DMA1_Channel1,10240);
			DMA_Cmd(DMA1_Channel1,ENABLE);
		}
		DMA_ClearITPendingBit(DMA1_IT_TC1);
	}
}
