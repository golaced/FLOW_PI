#ifndef _I2C_SOFT_H
#define	_I2C_SOFT_H

#include "stm32f10x.h"
#include "time.h"


#define SCL_H         ANO_GPIO_I2C->BSRR = I2C_Pin_SCL /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         ANO_GPIO_I2C->BRR  = I2C_Pin_SCL /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         ANO_GPIO_I2C->BSRR = I2C_Pin_SDA /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         ANO_GPIO_I2C->BRR  = I2C_Pin_SDA /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SCL /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SDA /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */
/***************I2C GPIO定义******************/
#define ANO_GPIO_I2C	GPIOB
#define I2C_Pin_SCL		GPIO_Pin_6
#define I2C_Pin_SDA		GPIO_Pin_7
#define ANO_RCC_I2C		RCC_APB2Periph_GPIOB
/*********************************************/
extern volatile u8 I2C_FastMode;

void I2c_Soft_Init(void);
void I2c_Soft_SendByte(u8 SendByte);
u8 I2c_Soft_ReadByte(u8);

//int I2c_Soft_Single_Write(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
//int I2c_Soft_Single_Read(u8 SlaveAddress,u8 REG_Address);
//int I2c_Soft_Mult_Read(u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size);

u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

#endif
