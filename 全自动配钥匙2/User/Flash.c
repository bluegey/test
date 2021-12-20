#include "Flash.h"
 
//��ȡָ����ַ�İ���(16λ����)
//addr:����ַ(�˵�ַ����Ϊ2�ı���!!)
//����ֵ:��Ӧ����.
uint16_t STM32_FLASH_ReadHalfWord(uint32_t addr)
{
	return *(vu16*)addr; 
}

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void STM32_FLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint16_t NumToRead)	
{
	uint16_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STM32_FLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
		ReadAddr+=2;//ƫ��2���ֽ�.	
	}
}

//ָ����ַд�����(16λ����)
//addr:д��ַ(�˵�ַ����Ϊ2�ı���!!)
//����ֵ:��Ӧ����.
void STM32_FLASH_WriteHalfWord(uint32_t addr,uint16_t value)
{
	FLASH_Unlock();
	
	FLASH_Lock();
}

//��ָ����ַ��ʼд��ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void STM32_FLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite)	
{
	uint16_t i;
	FLASH_Unlock();
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
	  WriteAddr+=2;//��ַ����2.
	} 
	FLASH_Lock();
}

void STM32_FLASH_EraseSector(uint32_t Sector_Address)
{
	FLASH_Unlock();
	FLASH_ErasePage(Sector_Address);
	FLASH_Lock();
}














