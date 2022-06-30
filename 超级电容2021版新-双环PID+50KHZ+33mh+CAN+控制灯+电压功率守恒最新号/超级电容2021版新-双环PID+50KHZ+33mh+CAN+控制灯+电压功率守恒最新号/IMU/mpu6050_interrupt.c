#include "init.h"
#include "stm32f10x_exti.h"
uint8_t isMPU6050_is_DRY = 0;   // mpu6050 interrupt中断标志

//中断设置引脚 
#define I2C_Pin_IT  GPIO_Pin_6
#define IT_line   EXTI_Line6
#define GPIO_PinSource   GPIO_PinSource6


void MPU6050_IntConfiguration(void)
{
		GPIO_InitTypeDef    gpio;
    EXTI_InitTypeDef    exti;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);   
		gpio.GPIO_Pin = I2C_Pin_IT ;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &gpio);
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource);

    exti.EXTI_Line = IT_line;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿中断
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    
}

//外部中断 6
uint8_t shoot_flag;
void EXTI9_5_IRQHandler(void)         //中断频率1KHz
{   
    if(EXTI_GetITStatus(EXTI_Line6) != RESET)
    {    
        EXTI_ClearFlag(EXTI_Line6);          
        EXTI_ClearITPendingBit(EXTI_Line6);
				Real_Monitor.IMUFrameCounter++;
        osSemaphoreRelease(GRYO_Semaphore);
    }

}

