#include "include.h"
#include "usart.h"

float dt_mems,dt_imu,dt_fushion,dt_uart;
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
		UART_PI_CONFIG(115200);
		UART_FLOW_CONFIG(115200);
		UART_UP_CONFIG(115200);
		MYDMA_Config(DMA1_Channel2,(u32)&USART3->DR,(u32)SendBuff,TEXT_LENTH);//DMA1通道4,外设为串口1,存储器为SendBuff,长(TEXT_LENTH+2)*100.
	  USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);      
   	MYDMA_Enable(DMA1_Channel2);  	
		//等待中断
		while(1)
		{
		dt_mems=(float)Get_Cycle_T(0)/1000000.;	
			
			
		MPU6050_Read(); 															
	  MPU6050_Data_Prepare( dt_mems );			
		
			
		//imu
    if(cnt[0]++>2){cnt[0]=0;		
		dt_imu=(float)Get_Cycle_T(1)/1000000.;		
		IMUupdate(0.5f *dt_imu,mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, 
			mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z,
	    0,0,0,
			&Roll,&Pitch,&Yaw);
		}
	  //fushion
		if(cnt[1]++>3){cnt[1]=0;		
		dt_fushion=(float)Get_Cycle_T(2)/1000000.;		
		
		}
		
		//uart
		if(cnt[2]++>3){cnt[2]=0;		
		dt_uart=(float)Get_Cycle_T(3)/1000000.;		
		
			
					if(DMA_GetFlagStatus(DMA1_FLAG_TC2)!=RESET)//等待通道4传输完成
					{
					DMA_ClearFlag(DMA1_FLAG_TC2);//清除通道4传输完成标志
					SendBuff_cnt=0;

					data_per_uart(
					0,0,0,
					0,0,0,
					0,0,0,
					(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),0,0,0/10,0);break;	
					}	
		}
		
		Delay_ms(3);
		}
}



