#ifndef __EXIT_H
#define __EXIT_H

#include "sys.h"

#define ADVANCE_EXTI_Linex EXTI_Line3
#define ADVANCE_EXTI_GPIO  GPIO_PortSourceGPIOE
#define ADVANCE_EXTI_PinSource GPIO_PinSource3

#define ADVANCE_NVIC_EXTI_Channel EXTI3_IRQn
void Exit_Init(void);
#endif
