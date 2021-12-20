#include "Flash.h"
 
//读取指定地址的半字(16位数据)
//addr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.
uint16_t STM32_FLASH_ReadHalfWord(uint32_t addr)
{
	return *(vu16*)addr; 
}

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STM32_FLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint16_t NumToRead)	
{
	uint16_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STM32_FLASH_ReadHalfWord(ReadAddr);//读取2个字节.
		ReadAddr+=2;//偏移2个字节.	
	}
}

//指定地址写入半字(16位数据)
//addr:写地址(此地址必须为2的倍数!!)
//返回值:对应数据.
void STM32_FLASH_WriteHalfWord(uint32_t addr,uint16_t value)
{
	FLASH_Unlock();
	
	FLASH_Lock();
}

//从指定地址开始写入指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STM32_FLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite)	
{
	uint16_t i;
	FLASH_Unlock();
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
	  WriteAddr+=2;//地址增加2.
	} 
	FLASH_Lock();
}

void STM32_FLASH_EraseSector(uint32_t Sector_Address)
{
	FLASH_Unlock();
	FLASH_ErasePage(Sector_Address);
	FLASH_Lock();
}














