/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
void Send_Massage(uint16_t ID,uint8_t *TX_Data);
void FDCAN1_Config(void);
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
typedef struct
{
  u16 super_volt;
  u16 power;
  u16 current;
}Tx_data;
typedef struct 
{
 uint16_t power_limit;
 uint16_t flag;
 uint16_t  Buffer_Energy;
}Re_Data;
typedef union 
{
  Re_Data  Rx_Data;
	uint8_t Array_RX_data[sizeof(Re_Data)];
}Can_Rx_Union;
/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
