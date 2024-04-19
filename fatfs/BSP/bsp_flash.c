#include "bsp_flash.h"
#include <stdbool.h>

#if STM32_FLASH_SIZE < 256
#define STM32_SECTOR_SIZE 1024 //页大小
#else
#define STM32_SECTOR_SIZE 2048 //页大小
#endif


bool mUseHalfWord;
uint32_t mStartAddress;
	
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];  //最多是2k,用来保存一页的数据


// STFLASH(uint32_t startAddress,bool useHalfWord)
// {
// 	if(startAddress%STM_SECTOR_SIZE!=0)
// 		startAddress+=(STM_SECTOR_SIZE-(startAddress%STM_SECTOR_SIZE));
// 	mStartAddress=startAddress;
// 	mUseHalfWord=useHalfWord;
// }
 
//读取指定地址的半字(16位数据)
//faddr:读地址(此地址必须是2的倍数)
//返回值:对应数据
uint16_t  ReadHalfWord(uint32_t faddr)
{
	return *(vu16*)faddr; 
}
//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void Flash_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
	
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
	  WriteAddr+=2;  //地址累加2
	}  
}
//带检查的写入,写入后读出检查
void Flash_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
	u32 secpos;	  //扇区地址
	u16 secoff;	   //扇区内偏移地址(16位字计算)
	u16 secremain; 	//扇区内剩余地址(16位字计算)   
 	u16 i;    
	u32 offaddr;   //去掉0x08000000后的地址
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return; //不在内存内的非法地址
	FLASH_Unlock();		//先解锁				
	offaddr=WriteAddr-STM32_FLASH_BASE;		//计算地址偏移
	secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址0~127;
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位)
	secremain=STM_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小
	if(NumToWrite<=secremain)secremain=NumToWrite; //不大于该扇区范围
	while(1) 
	{	
		Flash_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2); //读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;  //写入前必须是0xffff，否则就要擦除
		}
		if(i<secremain) //需要擦除
		{
			FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
			for(i=0;i<secremain;i++) //复制
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			Flash_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2); //写入整个扇区
		}else Flash_Write_NoCheck(WriteAddr,pBuffer,secremain);	//写已经擦除了的,直接写入扇区的剩余区间	   
		if(NumToWrite==secremain)break; //写入结束了
		else //写入未结束
		{
			secpos++;				//扇区地址增1
			secoff=0;				//偏移位置为0
		  pBuffer+=secremain; //指针偏移 	
			WriteAddr+=secremain;	//写地址偏移
		   	NumToWrite-=secremain;	//字节(16位)数递减
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2; //下一个扇区还是写不完
			else secremain=NumToWrite; //下一个扇区可以写完了
		}	 
	};	
	FLASH_Lock(); //上锁
}
 
//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void Flash_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead) 
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=ReadHalfWord(ReadAddr);
		ReadAddr+=2;
	}
}
