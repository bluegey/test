#ifndef __DEVICER_LED_H
#define __DEVICER_LED_H

#include "init.h"

//#define LED0_ON    GPIO_ResetBits(GPIOC,GPIO_Pin_0);
//#define LED0_OFF   GPIO_SetBits(GPIOC,GPIO_Pin_0);

//#define LED1_ON    GPIO_ResetBits(GPIOC,GPIO_Pin_1);
//#define LED1_OFF   GPIO_SetBits(GPIOC,GPIO_Pin_1);

//#define LED2_ON    GPIO_ResetBits(GPIOC,GPIO_Pin_2);
//#define LED2_OFF   GPIO_SetBits(GPIOC,GPIO_Pin_2);

//#define LED3_ON    GPIO_ResetBits(GPIOC,GPIO_Pin_3);
//#define LED3_OFF   GPIO_SetBits(GPIOC,GPIO_Pin_3);


#define LED0_ON    GPIO_ResetBits(GPIOB,GPIO_Pin_7);  //传感器2
#define LED0_OFF   GPIO_SetBits(GPIOB,GPIO_Pin_7);

#define LED1_ON    GPIO_ResetBits(GPIOB,GPIO_Pin_6);    //传感器1
#define LED1_OFF   GPIO_SetBits(GPIOB,GPIO_Pin_6);


void LED_Init(void);

#endif
