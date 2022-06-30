#include "OLED.h"
#include "Font.h"
#include "stdarg.h"
#include "delay.h"

#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

char GRAM[512]={0};

void OLED_WR_Byte(u8 dat,u8 cmd)
{	
	u8 i;			  
	if(cmd)
	  OLED_DC_Set();
	else
	  OLED_DC_Clr();
	OLED_CS_Clr();
	for(i=0;i<8;i++)
	{
		OLED_SCL_Clr();
		if(dat&0x80)
		   OLED_SDA_Set();
		else 
		   OLED_SDA_Clr();
		OLED_SCL_Set();
		dat<<=1;   
	}				 		  
	OLED_CS_Set();
	OLED_DC_Set();   	  
}

void WriteCmd(u8 Cmd)		//��������
{
#if	OLED_SEL_BUS
	OLED_WR_Byte(Cmd, 0);
#else
	IIC_Start();
	IIC_Write(0x78);			//Slave address
	IIC_Wait_Ack();	
	IIC_Write(0x00);			//write command
	IIC_Wait_Ack();	
	IIC_Write(Cmd); 
	IIC_Wait_Ack();	
	IIC_Stop();
#endif
}

void WriteDat(u8 Data)		//��������
{
#if	OLED_SEL_BUS
	OLED_WR_Byte(Data, 1);
#else
	IIC_Start();
	IIC_Write(0x78);			//Slave address
	IIC_Wait_Ack();	
	IIC_Write(0x40);			//write data
	IIC_Wait_Ack();	
	IIC_Write(Data); 
	IIC_Wait_Ack();	
	IIC_Stop();
#endif
}

void OLED_DBMP(int x0, int y0, int bmpx, int bmpy, const char *bmp)
{
	int dely, delly;
	int gcx, gcy, gcn;
	int gx, gy=0;
	
	gcn = (y0/8)*128+x0;				//��ʼ����
	gcx = (x0+bmpx)>127?128-x0:bmpx;	//x��Ч�ߴ�
	gcy = (y0+bmpy)>31?32-y0:bmpy;		//y��Ч�ߴ�
	dely = y0%8;						//yƫ����
	delly = 8-dely;
	
	if(gcx <= 0)
		return;							//xԽ��
	while(gcy > 0)
	{
		for(gx=0; gx<gcx; gx++)
			GRAM[gcn+gx] |= (bmp[gx+gy] << dely);
		gcn+=128;
		for(gx=0; gx<gcx; gx++)
			GRAM[gcn+gx] |= (bmp[gx+gy] >> delly);
		gy+=bmpx;
		gcy-=8;
	}
}

int OLED_FSizeSta(int set, int size)	//���������С״̬
{
	static int sta=0;
	if(set)
		sta = size;
	return sta;
}

void OLED_FSize(int size)
{
	OLED_FSizeSta(1, size);
}

void OLED_PStr(int x0, int y0, const char *str)
{
	int ch;
	int i=0;
	
	if(OLED_FSizeSta(0,0))
	{
		for(i=0; str[i]!='\0'; i++)
		{
			if(str[i] == '\n' || str[i] == '\r')
			{
				x0 = 0;
				y0 += 16;
				continue;
			}
			if(x0 > 120)
			{
				x0 = 0;
				y0 += 16;
			}
			ch = str[i] - 32;	//ASCII���������±����
			OLED_DBMP(x0, y0, 8, 16, &ASCII8x16[ch*16]);
			x0 += 8;
		}
	}
	else
	{
		for(i=0; str[i]!='\0'; i++)
		{
			if(str[i] == '\n' || str[i] == '\r')
			{
				x0 = 0;
				y0 += 8;
				continue;
			}
			if(x0 > 122)
			{
				x0 = 0;
				y0 += 8;
			}
			ch = str[i] - 32;
			OLED_DBMP(x0, y0, 6, 8, &ASCII6x8[ch*6]);
			x0 += 6;
		}
	}
}

void OLED_PCh(int x0, int y0, char ch)
{
	if(OLED_FSizeSta(0,0))
	{
		ch -= 32;
		OLED_DBMP(x0, y0, 8, 16, &ASCII8x16[ch*16]);
		x0 += 8;
	}
	else
	{
		ch -= 32;
		OLED_DBMP(x0, y0, 6, 8, &ASCII6x8[ch*6]);
		x0 += 6;
	}
}

void OLED_printf(int x0, int y0, const char *str, ...)
{
	char buf[128]={0};
	int i=0, n=0;
	
	//%s
	char *ptstr;
	char ptstri;
	
	//%d %f
	int ptint;
	int numn;
	float ptfloat;
	int ptfint;
	int ptfflt;
	int ptffs = 0; //С������
	
	va_list ap;
	__va_start(ap, str);
	
	while((str[i] != '\0')&&n<128)
	{
		ptffs = 7;
		if(str[i] == '%')
		{
			i++;
			switch(str[i])
			{
				case 'c':
					buf[n++] = __va_arg(ap, int);
					i++;
					continue;
				case 's':
					ptstr = __va_arg(ap, char*);
					ptstri = 0;
					while(ptstr[ptstri] != '\0')
						buf[n++] = ptstr[ptstri++];
					i++;
					continue;
				case 'd':
					ptint = __va_arg(ap,int);
					if(ptint == 0)
					{
						buf[n++] = '0';
						i++;
						continue;
					}
					if(ptint < 0)	//����
					{
						buf[n++] = '-';
						ptint = -ptint;
					}
					numn = 1000000000;	//int���2^32
					while(numn > ptint)numn/=10;
					while(numn != 0)
					{
						buf[n++] = (char)(ptint/numn+48);			//ASCII���������±����
						ptint %= numn;
						numn /= 10;
					}
					i++;
					continue;
				case '.':
					if(str[i+2] != 'f')break;
					ptffs = str[++i] - '0';
					i++;
				case 'f':
					ptfloat = __va_arg(ap,double);
					if(ptfloat < 0)
					{
						buf[n++] = '-';
						ptfloat = -ptfloat;
					}
					ptfint = (int)ptfloat;
					ptfflt = (int)(100000*(ptfloat-ptfint));
					if(ptfint == 0)										//��ӡ����
						buf[n++] = '0';
					else
					{
						numn = 1000000;
						while(numn > ptfint)numn/=10;
						while(numn != 0)
						{
							buf[n++] = (char)(ptfint/numn+48);
							ptfint %= numn;
							numn /= 10;
						}
					}
					buf[n++] = '.';
					if(ptfflt == 0)										//��ӡС��
						buf[n++] = '0';
					else
					{
						numn = 1000000;
						while(numn > ptfflt)numn/=10;
						while(numn != 0 && ptffs--)
						{
							buf[n++] = (char)(ptfflt/numn+48);
							ptfflt %= numn;
							numn /= 10;
						}
					}
					if(ptfloat < 0)
					{
						buf[n++] = '-';
						ptfloat = -ptfloat;
					}
					i++;
					continue;
			}
		}
		buf[n++] = str[i++];	//��ӡ�����ַ�
	}
	
	OLED_PStr(x0, y0, buf);
}

void OLED_DClean(void)	//��ջ���
{
	int i;
	for(i=0; i<512; i++)
		GRAM[i] = 0x00;
}

void OLED_Display(void)
{
	int i, n;
	for(i=0;i<4;i++)
	{
	   OLED_WR_Byte(0xb0+i,OLED_CMD); //��������ʼ��ַ
	   OLED_WR_Byte(0x00,OLED_CMD);   //���õ�����ʼ��ַ
	   OLED_WR_Byte(0x10,OLED_CMD);   //���ø�����ʼ��ַ
	   for(n=0;n<128;n++)
		 OLED_WR_Byte(GRAM[n+i*128],OLED_DATA);
  }
//	for(i = 0; i < 512; i++)
//		WriteDat(GRAM[i]);
}

void OLED_Clean(void)//����
{
	int i, n;
	for(i=0;i<4;i++)
	{
	   OLED_WR_Byte(0xb0+i,OLED_CMD); //��������ʼ��ַ
	   OLED_WR_Byte(0x00,OLED_CMD);   //���õ�����ʼ��ַ
	   OLED_WR_Byte(0x10,OLED_CMD);   //���ø�����ʼ��ַ
	   for(n=0;n<128;n++)
		 OLED_WR_Byte(GRAM[n+i*128],OLED_DATA);
  }
//	for(i = 0; i < 512; i++)
//		WriteDat(0x00);
}

void OLED_GPIO_Init(void)	//���ų�ʼ��
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//ʹ��PORTA~E,PORTGʱ��
  	
	//GPIO��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15 ;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��

	//GPIO��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
}

void OLED_Init_Setting(void)	//���ó�ʼ��
{
	/*
	WriteCmd(0xAE); //�ر���ʾ
	
	WriteCmd(0x20); //�����ڴ��ַģʽ    
	WriteCmd(0x00); //[1:0]��00��ˮƽ��ַģʽ;01����ֱ��ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
	
	WriteCmd(0xB0); //ҳ��ַģʽ��������ʼҳ0~7
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //�Աȶ�����
	WriteCmd(0xFF); //���ȵ��� 0x00~0xFF
	
	WriteCmd(0xA1); //���ض�������,bit0:0,0->0;1,0->127;
	WriteCmd(0xC8); //Set COM Output Scan Direction
	
	WriteCmd(0xA6); //--set normal display
	WriteCmd(0xA8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xD3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xD5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xF0); //--set divide ratio
	WriteCmd(0xD9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xDA); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xDB); //����VCOMH ��ѹ����
	WriteCmd(0x20); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
	
	WriteCmd(0x8D); //���õ�ɱ�
	WriteCmd(0x14); //bit2������/�ر�
	WriteCmd(0xAF); //������ʾ
	*/
	OLED_WR_Byte(0xAE,OLED_CMD); /*display off*/
	OLED_WR_Byte(0x00,OLED_CMD); /*set lower column address*/ 
	OLED_WR_Byte(0x10,OLED_CMD); /*set higher column address*/
	OLED_WR_Byte(0x00,OLED_CMD); /*set display start line*/ 
	OLED_WR_Byte(0xB0,OLED_CMD); /*set page address*/ 
	OLED_WR_Byte(0x81,OLED_CMD); /*contract control*/ 
	OLED_WR_Byte(0xff,OLED_CMD); /*128*/ 
	OLED_WR_Byte(0xA1,OLED_CMD); /*set segment remap*/ 
	OLED_WR_Byte(0xA6,OLED_CMD); /*normal / reverse*/ 
	OLED_WR_Byte(0xA8,OLED_CMD); /*multiplex ratio*/ 
	OLED_WR_Byte(0x1F,OLED_CMD); /*duty = 1/32*/ 
	OLED_WR_Byte(0xC8,OLED_CMD); /*Com scan direction*/ 
	OLED_WR_Byte(0xD3,OLED_CMD); /*set display offset*/ 
	OLED_WR_Byte(0x00,OLED_CMD); 
	OLED_WR_Byte(0xD5,OLED_CMD); /*set osc division*/ 
	OLED_WR_Byte(0x80,OLED_CMD); 
	OLED_WR_Byte(0xD9,OLED_CMD); /*set pre-charge period*/ 
	OLED_WR_Byte(0x1f,OLED_CMD); 
	OLED_WR_Byte(0xDA,OLED_CMD); /*set COM pins*/ 
	OLED_WR_Byte(0x00,OLED_CMD); 
	OLED_WR_Byte(0xdb,OLED_CMD); /*set vcomh*/ 
	OLED_WR_Byte(0x40,OLED_CMD); 
	OLED_WR_Byte(0x8d,OLED_CMD); /*set charge pump enable*/ 
	OLED_WR_Byte(0x14,OLED_CMD);
	OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/
}

void OLED_Init(void)
{
#if	OLED_SEL_BUS
	//SPI
	OLED_GPIO_Init();
	
	OLED_RES_Clr();
	delay_ms(200);
	OLED_RES_Set();
	
	OLED_CS_Set();
	OLED_DC_Set();
#else
	//IIC
	IIC_Init();
#endif
	
	//OLED����ʼ����ʱ
	//delay_ms(500);

	//OLED���ó�ʼ��
	OLED_Init_Setting();
}

