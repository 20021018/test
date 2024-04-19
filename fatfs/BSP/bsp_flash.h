#ifndef BSP_FLASH_H
#define BSP_FLASH_H
 
 
#include "ch32v30x.h"
#include "ch32v30x_flash.h"


#define STM32_FLASH_SIZE 256  	
#define STM32_FLASH_WREN 1


#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 
#else
#define STM_SECTOR_SIZE	2048
#endif		


#define STM32_FLASH_BASE 0x08000000 	





	
#ifdef __cplusplus
 extern "C" {
#endif

// 	STFLASH(uint32_t startAddress=(0x08000000+1000),bool useHalfWord=true);

u16 Flash_ReadHalfWord(u32 faddr);

void Flash_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite) ;

void Flash_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);
 
void Flash_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead) ;

 
 



#ifdef __cplusplus
 }
#endif



#endif





