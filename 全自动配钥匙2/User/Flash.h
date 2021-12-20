#ifndef __FLASH_H__
#define __FLASH_H__

#include <stm32f10x.h>


uint16_t STM32_FLASH_ReadHalfWord(uint32_t addr);
void STM32_FLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint16_t NumToRead);
void STM32_FLASH_WriteHalfWord(uint32_t addr,uint16_t value);
void STM32_FLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite);
void STM32_FLASH_EraseSector(uint32_t Sector_Address);
						   
#endif

















