#include "GT30L32S4W.h"
#include "Touch.h"
#include "mydelay.h"
#define SPI_FLASH_CS_LOW()       GPIO_ResetBits(GPIOG, GPIO_Pin_1)
#define SPI_FLASH_CS_HIGH()      GPIO_SetBits(GPIOG, GPIO_Pin_1)


/**********************模块命令********************/
#define Dummy_Byte                0xFF
#define	GT30L32S4W_ReadBytes			0x03
#define	GT30L32S4W_HSReadBytes  	0x0B

/**********************字库地址********************/
#define	GB_Size1112BaseAdd			0x000000
#define	GB_Size1516BaseAdd			0x02C9D0
#define	GB_Size2424BaseAdd			0x068190
#define	GB_Size3232BaseAdd			0x0EDF00
	
#define	ASCII_Size0507BaseAdd		0x1DDF80
#define	ASCII_Size0708BaseAdd		0x1DE280
#define	ASCII_Size0612BaseAdd		0x1DBE00
#define	ASCII_Size0816BaseAdd		0x1DD780
#define	ASCII_Size1224BaseAdd		0x1DFF00
#define	ASCII_Size1632BaseAdd		0x1E5A50

#define	ExChar_Size0612BaseAdd	0x1DBE0C
#define	ExChar_Size0816BaseAdd	0x1DD790
#define	ExChar_Size1224BaseAdd	0x1DFF30
#define	ExChar_Size1632BaseAdd	0x1E5A90


void SPI_FLASH_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;//时钟线
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;//输入
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;//输出
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//使能信号
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOG,&GPIO_InitStructure);

	SPI_FLASH_CS_HIGH();

	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial=7;
	SPI_Init(SPI2,&SPI_InitStructure);

	SPI_Cmd(SPI2,ENABLE);
}


uint8_t SPI_RW_Byte(uint8_t byte)
{
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(SPI2,byte);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
	return SPI_I2S_ReceiveData(SPI2);
}


uint32_t	GetChineseAddress(uint8_t type,uint8_t* GBCode,uint8_t* nbyte,uint8_t* ncols,uint8_t* nrows)
{
	uint32_t address=0;
	if(type==Chinese_1112)
	{
		if((GBCode[0] >=0xA1) && (GBCode[0] <= 0xA9) && (GBCode[1] >=0xA1)) 
			address =( (unsigned long)(GBCode[0] - 0xA1) * 94 + (GBCode[1] - 0xA1))*24+ GB_Size1112BaseAdd;
		else if((GBCode[0] >=0xB0) && (GBCode[0] <= 0xF7) && (GBCode[1] >=0xA1))   
			address = ((unsigned long)(GBCode[0] - 0xB0) * 94 + (GBCode[1] - 0xA1)+ 846)*24+ GB_Size1112BaseAdd;
		*nbyte=24;
		*ncols=11;
		*nrows=12; 	
	}

	else if(type==Chinese_1516)
	{
		if((GBCode[0] >=0xA1) && (GBCode[0] <= 0xA9) && (GBCode[1] >=0xA1)) 
			address =( (unsigned long)(GBCode[0] - 0xA1) * 94 + (GBCode[1] - 0xA1))*32+ GB_Size1516BaseAdd;
		else if((GBCode[0] >=0xB0) && (GBCode[0] <= 0xF7) && (GBCode[1] >=0xA1))   
			address = ((unsigned long)(GBCode[0] - 0xB0) * 94 + (GBCode[1] - 0xA1)+ 846)*32+ GB_Size1516BaseAdd;
		*nbyte=32;
		*ncols=15;
		*nrows=16;   	
	}

	else if(type==Chinese_2424)
	{
		if((GBCode[0] >=0xA1) && (GBCode[0] <= 0xA9) && (GBCode[1] >=0xA1)) 
			address =( (unsigned long)(GBCode[0] - 0xA1) * 94 + (GBCode[1] - 0xA1))*72+ GB_Size2424BaseAdd;
		else if((GBCode[0] >=0xB0) && (GBCode[0] <= 0xF7) && (GBCode[1] >=0xA1))   
			address = ((unsigned long)(GBCode[0] - 0xB0) * 94 + (GBCode[1] - 0xA1)+ 846)*72+ GB_Size2424BaseAdd;  	
		*nbyte=72;		
		*ncols=24;
		*nrows=24; 
	}

	else if(type==Chinese_3232)
	{
		if((GBCode[0] >=0xA1) && (GBCode[0] <= 0xA9) && (GBCode[1] >=0xA1)) 
			address =( (unsigned long)(GBCode[0] - 0xA1) * 94 + (GBCode[1] - 0xA1))*128+ GB_Size3232BaseAdd;
		else if((GBCode[0] >=0xB0) && (GBCode[0] <= 0xF7) && (GBCode[1] >=0xA1))   
			address = ((unsigned long)(GBCode[0] - 0xB0) * 94 + (GBCode[1] - 0xA1)+ 846)*128+ GB_Size3232BaseAdd;  	
		*nbyte=128;
		*ncols=32;
		*nrows=32; 
	}

	else if(type==ASCII_0507)
	{
		if ((GBCode[0] >= 0x20) && (GBCode[0] <= 0x7E)) 
			address = (unsigned long)(GBCode[0]-0x20 ) * 8+ASCII_Size0507BaseAdd;  	
		*nbyte=8;
		*ncols=5;
		*nrows=8; 
	}

	else if(type==ASCII_0708)
	{
		if ((GBCode[0] >= 0x20) && (GBCode[0] <= 0x7E)) 
			address = (unsigned long)(GBCode[0]-0x20 ) * 8+ASCII_Size0708BaseAdd;  	
		*nbyte=8;
		*ncols=7;
		*nrows=8; 
	}

	else if(type==ASCII_0612)
	{
		if ((GBCode[0] >= 0x20) && (GBCode[0] <= 0x7E))
			address = (unsigned long)(GBCode[0]-0x20 ) * 12+ASCII_Size0612BaseAdd;  	
		*nbyte=12;
		*ncols=6;
		*nrows=12; 
	}

	else if(type==ASCII_0816)
	{
		if ((GBCode[0] >= 0x20) && (GBCode[0] <= 0x7E)) 
			address = (unsigned long)(GBCode[0]-0x20 ) * 16+ASCII_Size0816BaseAdd;  	
		*nbyte=16;
		*ncols=8;
		*nrows=16; 
	}

	else if(type==ASCII_1224)
	{
		if ((GBCode[0] >= 0x20) && (GBCode[0] <= 0x7E)) 
			address = (unsigned long)(GBCode[0]-0x20 ) * 48+ASCII_Size1224BaseAdd;  	
		*nbyte=48;
		*ncols=12;
		*nrows=24; 
	}

	else if(type==ASCII_1632)
	{
		if ((GBCode[0] >= 0x20) && (GBCode[0] <= 0x7E)) 
			address = (unsigned long)(GBCode[0]-0x20 ) * 64+ASCII_Size1632BaseAdd;  	
		*nbyte=64;
		*ncols=16;
		*nrows=32; 
	}
	
	return address;	
} 



void GT30L32S4W_Read(uint32_t address,uint8_t nbyte,uint8_t* pBuffer)
{
	uint8_t i;
	if(TP_enable_irq_flag)				//关闭触控中断
		NVIC_DisableIRQ(EXTI1_IRQn);
	
	SPI_FLASH_CS_LOW();
	SPI_RW_Byte(GT30L32S4W_HSReadBytes);
		my_delayus(6);//ADS7846的转换时间最长为6us 	    
	SPI_RW_Byte((address & 0xFFFFFF) >> 16);
	SPI_RW_Byte((address & 0x00FFFF) >> 8);
	SPI_RW_Byte(address & 0x0000FF);
	SPI_RW_Byte(Dummy_Byte);

	for(i=0;i<nbyte;i++)
		*pBuffer++=SPI_RW_Byte(Dummy_Byte);	

	SPI_FLASH_CS_HIGH();
	
	if(TP_enable_irq_flag)				//使能触控中断
		NVIC_EnableIRQ(EXTI1_IRQn);
}




