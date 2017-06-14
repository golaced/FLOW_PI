#include "include.h"

float dt_mems,dt_imu;
int main(void)
{static u8 cnt[10];
		//--------------------------- CLK INIT, HSE PLL ----------------------------
		ErrorStatus HSEStartUpStatus;
		//RCC reset
		RCC_DeInit();
		//开启外部时钟 并执行初始化
		RCC_HSEConfig(RCC_HSE_ON); 
		//等待外部时钟准备好
		HSEStartUpStatus = RCC_WaitForHSEStartUp();
		//启动失败 在这里等待
		while(HSEStartUpStatus == ERROR);
		//设置内部总线时钟
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		//外部时钟为8M 这里倍频到72M
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE); 
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);

		//----------------------------- CLOSE HSI ---------------------------
		//关闭内部时钟HSI
		RCC_HSICmd(DISABLE);	
		
		//开PA口时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			
		//中断配置 2-level interrupt 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		
		//开总中断
		__enable_irq(); 
		
	  delay_init(72);
		TIM5_Config();
		Cycle_Time_Init();
		MPU6050_Init(20);
		//等待中断
		while(1)
		{
		dt_mems=(float)Get_Cycle_T(0)/1000000.;	
			
			
		MPU6050_Read(); 															
	  MPU6050_Data_Prepare( dt_mems );			
		
			
			
    if(cnt[0]++>2){cnt[0]=0;		
		dt_imu=(float)Get_Cycle_T(1)/1000000.;		
		IMUupdate(0.5f *dt_imu,mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z,
	    0,0,0,
			&Roll,&Pitch,&Yaw);
		}
	
		
		
		Delay_ms(3);
		}
}



