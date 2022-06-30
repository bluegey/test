/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       main.c/h
  * @brief
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     	2021     			STORMS       1. Íê³É
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */
#include "main.h"

int main(void)
{
     BSP_Init();
    delay_ms(2500);
    startTask();
    while(1);
}
