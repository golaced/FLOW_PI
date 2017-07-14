
#include "stm32f10x.h"
#include "time.h"


#define SCL_H_VL53         ANO_GPIO_I2C_VL53->BSRR = I2C_Pin_SCL_VL53 /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L_VL53         ANO_GPIO_I2C_VL53->BRR  = I2C_Pin_SCL_VL53 /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H_VL53         ANO_GPIO_I2C_VL53->BSRR = I2C_Pin_SDA_VL53 /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L_VL53         ANO_GPIO_I2C_VL53->BRR  = I2C_Pin_SDA_VL53 /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read_VL53      ANO_GPIO_I2C_VL53->IDR  & I2C_Pin_SCL_VL53 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read_VL53      ANO_GPIO_I2C_VL53->IDR  & I2C_Pin_SDA_VL53 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */
/***************I2C GPIO??******************/
#define ANO_GPIO_I2C_VL53	GPIOB
#define I2C_Pin_SCL_VL53		GPIO_Pin_0
#define I2C_Pin_SDA_VL53		GPIO_Pin_1
#define ANO_RCC_I2C_VL53	RCC_APB2Periph_GPIOB
/*********************************************/
extern volatile u8 I2C_FastMode_VL53;

void I2c_Soft_Init_VL53(void);
void I2c_Soft_SendByte_VL53(u8 SendByte);
u8 I2c_Soft_ReadByte_VL53(u8);

//int I2c_Soft_Single_Write(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
//int I2c_Soft_Single_Read(u8 SlaveAddress,u8 REG_Address);
//int I2c_Soft_Mult_Read(u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size);

u8 IIC_Write_1Byte_VL53(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte_VL53(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_Write_nByte_VL53(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte_VL53(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define VL53L0X_Add 0x29


typedef struct 
{ 
uint16_t ambient;
uint16_t signal ;
uint16_t distance; 
uint16_t mode;
}LS53;

extern LS53 ls53;

u8 READ_VL53(void);