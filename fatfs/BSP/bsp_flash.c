#include "bsp_flash.h"
#include <stdbool.h>

#if STM32_FLASH_SIZE < 256
#define STM32_SECTOR_SIZE 1024 //ҳ��С
#else
#define STM32_SECTOR_SIZE 2048 //ҳ��С
#endif


bool mUseHalfWord;
uint32_t mStartAddress;
	
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];  //�����2k,��������һҳ������


// STFLASH(uint32_t startAddress,bool useHalfWord)
// {
// 	if(startAddress%STM_SECTOR_SIZE!=0)
// 		startAddress+=(STM_SECTOR_SIZE-(startAddress%STM_SECTOR_SIZE));
// 	mStartAddress=startAddress;
// 	mUseHalfWord=useHalfWord;
// }
 
//��ȡָ����ַ�İ���(16λ����)
//faddr:����ַ(�˵�ַ������2�ı���)
//����ֵ:��Ӧ����
uint16_t  ReadHalfWord(uint32_t faddr)
{
	return *(vu16*)faddr; 
}
//������д��
//WriteAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void Flash_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
	
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
	  WriteAddr+=2;  //��ַ�ۼ�2
	}  
}
//������д��,д���������
void Flash_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
	u32 secpos;	  //������ַ
	u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
	u16 secremain; 	//������ʣ���ַ(16λ�ּ���)   
 	u16 i;    
	u32 offaddr;   //ȥ��0x08000000��ĵ�ַ
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return; //�����ڴ��ڵķǷ���ַ
	FLASH_Unlock();		//�Ƚ���				
	offaddr=WriteAddr-STM32_FLASH_BASE;		//�����ַƫ��
	secpos=offaddr/STM_SECTOR_SIZE;			//������ַ0~127;
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ)
	secremain=STM_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С
	if(NumToWrite<=secremain)secremain=NumToWrite; //�����ڸ�������Χ
	while(1) 
	{	
		Flash_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2); //������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;  //д��ǰ������0xffff�������Ҫ����
		}
		if(i<secremain) //��Ҫ����
		{
			FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//�����������
			for(i=0;i<secremain;i++) //����
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			Flash_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2); //д����������
		}else Flash_Write_NoCheck(WriteAddr,pBuffer,secremain);	//д�Ѿ������˵�,ֱ��д��������ʣ������	   
		if(NumToWrite==secremain)break; //д�������
		else //д��δ����
		{
			secpos++;				//������ַ��1
			secoff=0;				//ƫ��λ��Ϊ0
		  pBuffer+=secremain; //ָ��ƫ�� 	
			WriteAddr+=secremain;	//д��ַƫ��
		   	NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2; //��һ����������д����
			else secremain=NumToWrite; //��һ����������д����
		}	 
	};	
	FLASH_Lock(); //����
}
 
//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void Flash_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead) 
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=ReadHalfWord(ReadAddr);
		ReadAddr+=2;
	}
}
