#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "include.h"   
#include "stm32f10x_flash.h"
void READ_PARM(void);
void WRITE_PARM(void);
				

#define FLASH_START_ADDR    ((uint32_t)0x8000000)  
#define FLASH_END_ADDR      ((uint32_t)(0x8000000 + FLASH_SECTOR_NUM * FLASH_SECTOR_SIZE))  
#define FLASH_SECTOR_NUM    128  // ????  
#define FLASH_SECTOR_SIZE   1024 // ????1KB  
  
typedef enum  
{   
  FLASH_FAILURE = 0,  
  FLASH_SUCCESS,  
} flash_status_t;  
  
//===========================================================================  
flash_status_t FlashErase(uint32_t addr, uint8_t count);  
uint32_t FlashWrite(uint32_t addr, uint8_t *buffer, uint32_t length);  
uint32_t FlashRead(uint32_t addr, uint8_t *buffer, uint32_t length);  				
#endif

















