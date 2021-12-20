#ifndef __GPIO__H
#define __GPIO__H

#include "stm32f4xx_it.h"

#define RED_ON GPIO_SetBits(GPIOA, GPIO_Pin_15);
#define RED_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_15);

#define GREEN_ON GPIO_SetBits(GPIOA, GPIO_Pin_5);
#define GREEN_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_5);

void gpio_init(void);

#endif

