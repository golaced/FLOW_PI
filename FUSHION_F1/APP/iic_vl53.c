
#include "iic_vl53.h"
#include "delay.h"
#include "filter.h"
volatile u8 I2C_FastMode_VL53;
LS53 ls53;
void I2c_Soft_delay_VL53()
{ 
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	
	if(!I2C_FastMode_VL53)
	{
		u8 i = 15;
		while(i--);
	}
}
//u8 IIC_Write_1Byte_VL53(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
//u8 IIC_Read_1Byte_VL53(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
//u8 IIC_Write_nByte_VL53(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
//u8 IIC_Read_nByte_VL53(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

uint16_t makeuint16(int lsb, int msb)
{
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

u8 READ_VL53(void){
u8 flag1=0,val=0;
u8 gbuf[16];
uint16_t count[3];
u8 DeviceRangeStatusInternal;
		 IIC_Write_1Byte_VL53(VL53L0X_Add,VL53L0X_REG_SYSRANGE_START, 0x01);
		 
		 while(flag1 < 10)
		 {
				Delay_ms(1);
				IIC_Read_1Byte_VL53(VL53L0X_Add,VL53L0X_REG_RESULT_RANGE_STATUS,&val);
				if( val & 0x01) break;
				flag1++;
		 }

		 if( val & 0x01)
		 {
		 IIC_Read_nByte_VL53(VL53L0X_Add, 0x14 , 12, gbuf);	
		 count[0] = makeuint16(gbuf[7], gbuf[6]);
		 count[1] = makeuint16(gbuf[9], gbuf[8]);
		 count[2] = makeuint16(gbuf[11], gbuf[10]);
		 DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);
			 
			ls53.ambient= count[0];
			ls53.signal= count[1];
			if(count[2]<1800){ 
			if(count[2]!=20)
			ls53.distance= insert_sonar_value_and_get_mode_value(count[2]);//Moving_Median(1,5,temp);count[2];
			ls53.mode=1;//= DeviceRangeStatusInternal;			 
				return 1;//printf("\r\n readey \r\n");
		  }else{
				ls53.mode=2;
        return 2;		
			}				
		 }
		 else
		 {ls53.mode=0;
				return 0;}//printf("\r\n not readey \r\n");
		// printf("\r\n ambient count = %4d signal count = %4d distance = %4d status = %d ",count[0],count[1],count[2],DeviceRangeStatusInternal);
		// data_analy(count[2]);
	 }	 
	
	
void I2c_Soft_Init_VL53()
{
	
	GPIO_InitTypeDef  GPIO_InitStructure; 
	RCC_APB2PeriphClockCmd(ANO_RCC_I2C_VL53, ENABLE);  //Ê¹ÄÜGPIOBÊ±ÖÓ
	GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
  GPIO_Init(ANO_GPIO_I2C_VL53, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(ANO_GPIO_I2C_VL53, &GPIO_InitStructure);
}

int I2c_Soft_Start_VL53()
{
	SDA_H_VL53;
	SCL_H_VL53;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53)return 0;	//SDA?????????,??
	SDA_L_VL53;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53) return 0;	//SDA??????????,??
	SDA_L_VL53;
	I2c_Soft_delay_VL53();
	return 1;	

}

void I2c_Soft_Stop_VL53()
{
	SCL_L_VL53;
	I2c_Soft_delay_VL53();
	SDA_L_VL53;
	I2c_Soft_delay_VL53();
	SCL_H_VL53;
	I2c_Soft_delay_VL53();
	SDA_H_VL53;
	I2c_Soft_delay_VL53();
}

void I2c_Soft_Ask_VL53()
{
	SCL_L_VL53;
	I2c_Soft_delay_VL53();
	SDA_L_VL53;
	I2c_Soft_delay_VL53();
	SCL_H_VL53;
	I2c_Soft_delay_VL53();
	SCL_L_VL53;
	I2c_Soft_delay_VL53();
}

void I2c_Soft_NoAsk_VL53()
{
	SCL_L_VL53;
	I2c_Soft_delay_VL53();
	SDA_H_VL53;
	I2c_Soft_delay_VL53();
	SCL_H_VL53;
	I2c_Soft_delay_VL53();
	SCL_L_VL53;
	I2c_Soft_delay_VL53();
}

int I2c_Soft_WaitAsk_VL53(void) 	 //???:=1?ASK,=0?ASK
{
  u8 ErrTime = 0;
	SCL_L_VL53;
	I2c_Soft_delay_VL53();
	SDA_H_VL53;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53();
			return 1;
		}
	}
	SCL_L_VL53;
	I2c_Soft_delay_VL53();
	return 0;
}

void I2c_Soft_SendByte_VL53(u8 SendByte) //????????//
{
    u8 i=8;
    while(i--)
    {
        SCL_L_VL53;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53;  
      else 
        SDA_L_VL53;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53;
}  

//?1???,ack=1?,??ACK,ack=0,??NACK
u8 I2c_Soft_ReadByte_VL53(u8 ask)  //????????//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H_VL53;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53;
      I2c_Soft_delay_VL53();
			SCL_H_VL53;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53;

	if (ask)
		I2c_Soft_Ask_VL53();
	else
		I2c_Soft_NoAsk_VL53();  
    return ReceiveByte;
} 


// IIC???????
u8 IIC_Write_1Byte_VL53(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
	I2c_Soft_Start_VL53();
	I2c_Soft_SendByte_VL53(SlaveAddress<<1);   
	if(I2c_Soft_WaitAsk_VL53())
	{
		I2c_Soft_Stop_VL53();
		return 1;
	}
	I2c_Soft_SendByte_VL53(REG_Address);       
	I2c_Soft_WaitAsk_VL53();	
	I2c_Soft_SendByte_VL53(REG_data);
	I2c_Soft_WaitAsk_VL53();   
	I2c_Soft_Stop_VL53(); 
	return 0;
}

// IIC?1????
u8 IIC_Read_1Byte_VL53(u8 SlaveAddress,u8 REG_Address,u8 *REG_data)
{      		
	I2c_Soft_Start_VL53();
	I2c_Soft_SendByte_VL53(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk_VL53())
	{
		I2c_Soft_Stop_VL53();
		return 1;
	}
	I2c_Soft_SendByte_VL53(REG_Address);     
	I2c_Soft_WaitAsk_VL53();
	I2c_Soft_Start_VL53();
	I2c_Soft_SendByte_VL53(SlaveAddress<<1 | 0x01);
	I2c_Soft_WaitAsk_VL53();
	*REG_data= I2c_Soft_ReadByte_VL53(0);
	I2c_Soft_Stop_VL53();
	return 0;
}	

// IIC?n????
u8 IIC_Write_nByte_VL53(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2c_Soft_Start_VL53();
	I2c_Soft_SendByte_VL53(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk_VL53())
	{
		I2c_Soft_Stop_VL53();
		return 1;
	}
	I2c_Soft_SendByte_VL53(REG_Address); 
	I2c_Soft_WaitAsk_VL53();
	while(len--) 
	{
		I2c_Soft_SendByte_VL53(*buf++); 
		I2c_Soft_WaitAsk_VL53();
	}
	I2c_Soft_Stop_VL53();
	return 0;
}

// IIC?n????
u8 IIC_Read_nByte_VL53(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2c_Soft_Start_VL53();
	I2c_Soft_SendByte_VL53(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk_VL53())
	{
		I2c_Soft_Stop_VL53();
		return 1;
	}
	I2c_Soft_SendByte_VL53(REG_Address); 
	I2c_Soft_WaitAsk_VL53();
	
	I2c_Soft_Start_VL53();
	I2c_Soft_SendByte_VL53(SlaveAddress<<1 | 0x01); 
	I2c_Soft_WaitAsk_VL53();
	while(len) 
	{
		if(len == 1)
		{
			*buf = I2c_Soft_ReadByte_VL53(0);
		}
		else
		{
			*buf = I2c_Soft_ReadByte_VL53(1);
		}
		buf++;
		len--;
	}
	I2c_Soft_Stop_VL53();
	return 0;
}


