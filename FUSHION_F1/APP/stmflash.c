#include "stmflash.h"
 
// addr:??  count:???  
flash_status_t FlashErase(uint32_t addr, uint8_t count)  
{  
  uint8_t i;  
  
  for(i = 0; i < count; ++i)  
  {  
    if(FLASH_ErasePage(addr + i * FLASH_SECTOR_SIZE) != FLASH_COMPLETE)  
    {  
      return FLASH_FAILURE;  
    }  
  }  
  
  return FLASH_SUCCESS;  
}  
  
uint32_t FlashWrite(uint32_t addr, uint8_t *buffer, uint32_t length)  
{  
  uint16_t i, data = 0;  
  
  FLASH_Unlock();  
  
  for(i = 0; i < length; i += 2)  
  {  
    data = (*(buffer + i + 1) << 8) + (*(buffer + i));  
    if(FLASH_ProgramHalfWord((uint32_t)(addr + i), data) != FLASH_COMPLETE)  
    {  
      return i;  
    }  
  }  
    
  FLASH_Lock();  
  
  return length;  
}  
  
uint32_t FlashRead(uint32_t addr, uint8_t *buffer, uint32_t length)  
{  
  memcpy(buffer, (void *)addr, length);  
  
  return length;  
}  

//-----------------------------------------´æ´¢²ÎÊı
#define FLASH_USE_STM32 1
#define SIZE_PARAM 50
u8 FLASH_READ_BUF[SIZE_PARAM]={0};
void READ_PARM(void)
{

FlashRead(FLASH_END_ADDR,FLASH_READ_BUF,SIZE_PARAM);	

mpu6050_fc.Gyro_Offset.x=(vs16)(FLASH_READ_BUF[1]<<8|FLASH_READ_BUF[0]);
mpu6050_fc.Gyro_Offset.y=(vs16)(FLASH_READ_BUF[3]<<8|FLASH_READ_BUF[2]);
mpu6050_fc.Gyro_Offset.z=(vs16)(FLASH_READ_BUF[5]<<8|FLASH_READ_BUF[4]);
	
mpu6050_fc.Acc_Offset.x=(vs16)(FLASH_READ_BUF[7]<<8|FLASH_READ_BUF[6]);
mpu6050_fc.Acc_Offset.y=(vs16)(FLASH_READ_BUF[9]<<8|FLASH_READ_BUF[8]);
mpu6050_fc.Acc_Offset.z=(vs16)(FLASH_READ_BUF[11]<<8|FLASH_READ_BUF[10]);
	
//ak8975_fc.Mag_Offset.x=(vs16)(FLASH_READ_BUF[13]<<8|FLASH_READ_BUF[12]);
//ak8975_fc.Mag_Offset.y=(vs16)(FLASH_READ_BUF[15]<<8|FLASH_READ_BUF[14]);
//ak8975_fc.Mag_Offset.z=(vs16)(FLASH_READ_BUF[17]<<8|FLASH_READ_BUF[16]);
//	
//ak8975_fc.Mag_Gain.x =(float)((vs16)((FLASH_READ_BUF[19]<<8|FLASH_READ_BUF[18])))/100.;
//ak8975_fc.Mag_Gain.y=(float)((vs16)((FLASH_READ_BUF[21]<<8|FLASH_READ_BUF[20])))/100.;
//ak8975_fc.Mag_Gain.z =(float)((vs16)((FLASH_READ_BUF[23]<<8|FLASH_READ_BUF[22])))/100.;
	

	 WRITE_PARM();
}

void WRITE_PARM(void)
{ 

int16_t _temp;
u8 cnt=0;

_temp=(int16_t)mpu6050_fc.Gyro_Offset.x;
FLASH_READ_BUF[cnt++]=BYTE0(_temp);
FLASH_READ_BUF[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050_fc.Gyro_Offset.y;
FLASH_READ_BUF[cnt++]=BYTE0(_temp);
FLASH_READ_BUF[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050_fc.Gyro_Offset.z;
FLASH_READ_BUF[cnt++]=BYTE0(_temp);
FLASH_READ_BUF[cnt++]=BYTE1(_temp);

_temp=(int16_t)mpu6050_fc.Acc_Offset.x;
FLASH_READ_BUF[cnt++]=BYTE0(_temp);
FLASH_READ_BUF[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050_fc.Acc_Offset.y;
FLASH_READ_BUF[cnt++]=BYTE0(_temp);
FLASH_READ_BUF[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050_fc.Acc_Offset.z;
FLASH_READ_BUF[cnt++]=BYTE0(_temp);
FLASH_READ_BUF[cnt++]=BYTE1(_temp);
//_temp=(int16_t)ak8975_fc.Mag_Offset.x;
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);
//_temp=(int16_t)ak8975_fc.Mag_Offset.y;
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);
//_temp=(int16_t)ak8975_fc.Mag_Offset.z;
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);


//_temp=(int16_t)(ak8975_fc.Mag_Gain.x*100);
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);
//_temp=(int16_t)(ak8975_fc.Mag_Gain.y*100);
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);
//_temp=(int16_t)(ak8975_fc.Mag_Gain.z*100);
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);

FlashWrite(FLASH_START_ADDR,FLASH_READ_BUF,SIZE_PARAM);

}
