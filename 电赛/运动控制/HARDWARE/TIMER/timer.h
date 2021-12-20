#ifndef __TIMER__H
#define __TIMER__H

#include "stm32f4xx_it.h"
void TIM1_Int_Init(u16 arr, u16 psc);
void TIM5_PWM_Init(u32 arr, u32 psc);
void TIM3_PWM_Init(u32 arr, u32 psc);
void TIM4_PWM_Init(u32 arr, u32 psc);
#endif
