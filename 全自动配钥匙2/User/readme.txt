��Ƭ��IO���ŷ����
LCD��	ʹ�õ�Ƭ���ڲ�FSMC���ƣ�FSMC_Bank1_NORSRAM4
			LCD-D0~D15:	PD14��15��0��1��PE7��8��9��10��11��12��13��14��15��PD8��9��10
			LCD-RD:			PD4-FSMC_NOE
			LCD-WR: 		PD5-FSMC_NWE
			LCD-CS:		 	PG12-FSMC_NE4
			LCD-DC:			PD11-FSMC_A16
			LCD-RST:		PA3
			LCD-BL:			PG7
Touch:ʹ�õ�Ƭ���ڲ�SPI2ͨ�ţ�ʹ���ж�
			TP-CS:			PB12
			TP-CLK:			PB13
			TP-MOSI:		PB15
			TP-MISO:		PB14
			TP-IRQ:			PA1
FLASH:ʹ�õ�Ƭ���ڲ�SPI2ͨ��,���봥��ʹ�ù���һ·SPI���ߣ���ȡ����ʱҪ�رմ����ж�
			FLASH-CS:		PG1	
			FLASH-CLK:	PB13
			FLASH-MOSI:	PB15
			FLASH-MISO:	PB14
EEPROM:ʹ��ģ��IIC����ͨ�ţ����ڴ洢����������ͼ�����
			SCL:				PB6
			SDA:				PB7
SDcard:ʹ�õ�Ƭ���ڲ�SDIOģ��
			SDIO_D0:		PC8
			SDIO_D1:		PC9
			SDIO_D2:		PC10
			SDIO_D3:		PC11
			SDIO_CLK:		PC12
			SDIO_CMD:		PD2
UART:	ʹ�ô���1��������115200
			TXD��				PA9
			RXD:				PA10
Camera:ʹ��SCCB�������üĴ�����ʹ��TIM4_1���岶׽����DMA�ɼ����ݣ�ʹ���г�����ź�
			SCCB_SDA:		PF8
			SCCB_SCL:		PF9
			VREF:				PC0
			HREF:				PC1
			PLL:				PA2
			D0~D7:			PF0~PF7
LED:	ʹ��1·PWM����ͼ��ɼ��ƹ�����
			LED-PWM:		PB1
StepMotor:ʹ��4·�������ƽ̨ǰ�������ƶ���2·���������Ʋ������΢��
			Motor_F:		PG2		
			Motor_B:		PG3
			Motor_L:		PG5
			Motor_R:		PG6
			adjust_FB:	PG4
			adjust_LF:	PG8
Motor:1·������ڿ����и���
			Motor_SW:		PF15
Switch:1·������ڿ���ϵͳ��ͣ��ť
			Main_SW:		PG0
Input:4·��λ�������룬Ԥ��1·			
			Limit_F:		PF13
			Limit_B:		PF11
			Limit_L:		PF14
			Limit_R:		PF12
			


ͼ��洢��SD����ʱ�����÷�˫������ṹ����SD���������������𻵵���ͼ��д���ȡʧ�ܡ�
�洢һ��Կ��������400KB������ǰ300KB���ڴ洢ͼ�����ݣ���100KB����ͼ����Ϣ�ṹ�����ݡ�
EEPROM���ڱ��津��������������ͼ�����ݴ洢�ṹ�塣
			