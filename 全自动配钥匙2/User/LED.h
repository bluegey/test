#ifndef	_LED_H_	
#define	_LED_H_

#include "stm32f10x.h"

#define LED1_PORT		 	GPIOD
#define LED1_PIN	    GPIO_Pin_14
#define LED2_PORT		 	GPIOG
#define LED2_PIN	    GPIO_Pin_13


#define LED1_ON()        {GPIO_ResetBits(LED1_PORT,LED1_PIN);}
#define LED1_OFF()       {GPIO_SetBits(LED1_PORT,LED1_PIN);}
#define LED2_ON()        {GPIO_ResetBits(LED2_PORT,LED2_PIN);}
#define LED2_OFF()       {GPIO_SetBits(LED2_PORT,LED2_PIN);}

void LED_Init(void);

#endif
