#ifndef __DEVICER_PWM_H
#define __DEVICER_PWM_H

#include "init.h"



#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )


void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM2_PWM_Init(u16 hz);


void Motor_Init(void);
void Device_Stop(void);
void Device_Rollback(void);
void Device_Corotation(void);
void PWM_GPIO_Init(void);

void Device_corotation(void);
void Device_rollback(void);
void Device_stop(void);

void Change_Motor_PWM(uint8_t num,u16 duty);
void Change_RotatePWM(void);

void Change_Motor_pwm(uint8_t num,u16 duty);
void Change_Rotatepwm(void);
#endif
