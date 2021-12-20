单片机IO引脚分配表
LCD：	使用单片机内部FSMC控制，FSMC_Bank1_NORSRAM4
			LCD-D0~D15:	PD14、15、0、1、PE7、8、9、10、11、12、13、14、15、PD8、9、10
			LCD-RD:			PD4-FSMC_NOE
			LCD-WR: 		PD5-FSMC_NWE
			LCD-CS:		 	PG12-FSMC_NE4
			LCD-DC:			PD11-FSMC_A16
			LCD-RST:		PA3
			LCD-BL:			PG7
Touch:使用单片机内部SPI2通信，使用中断
			TP-CS:			PB12
			TP-CLK:			PB13
			TP-MOSI:		PB15
			TP-MISO:		PB14
			TP-IRQ:			PA1
FLASH:使用单片机内部SPI2通信,因与触控使用共用一路SPI总线，读取数据时要关闭触控中断
			FLASH-CS:		PG1	
			FLASH-CLK:	PB13
			FLASH-MOSI:	PB15
			FLASH-MISO:	PB14
EEPROM:使用模拟IIC进行通信，用于存储触屏参数，图像参数
			SCL:				PB6
			SDA:				PB7
SDcard:使用单片机内部SDIO模块
			SDIO_D0:		PC8
			SDIO_D1:		PC9
			SDIO_D2:		PC10
			SDIO_D3:		PC11
			SDIO_CLK:		PC12
			SDIO_CMD:		PD2
UART:	使用串口1，波特率115200
			TXD：				PA9
			RXD:				PA10
Camera:使用SCCB总线配置寄存器，使用TIM4_1脉冲捕捉触发DMA采集数据，使用行场输出信号
			SCCB_SDA:		PF8
			SCCB_SCL:		PF9
			VREF:				PC0
			HREF:				PC1
			PLL:				PA2
			D0~D7:			PF0~PF7
LED:	使用1路PWM调节图像采集灯光亮度
			LED-PWM:		PB1
StepMotor:使用4路输出控制平台前后左右移动，2路输出输出控制步进电机微调
			Motor_F:		PG2		
			Motor_B:		PG3
			Motor_L:		PG5
			Motor_R:		PG6
			adjust_FB:	PG4
			adjust_LF:	PG8
Motor:1路输出用于开关切割电机
			Motor_SW:		PF15
Switch:1路输出用于控制系统急停按钮
			Main_SW:		PG0
Input:4路限位开关输入，预留1路			
			Limit_F:		PF13
			Limit_B:		PF11
			Limit_L:		PF14
			Limit_R:		PF12
			


图像存储到SD卡中时，采用仿双向链表结构，因SD卡个别扇区可能损坏导致图像写入读取失败。
存储一幅钥匙数据用400KB，其中前300KB用于存储图像数据，后100KB用于图像信息结构体数据。
EEPROM用于保存触屏参数，还保存图像数据存储结构体。
			