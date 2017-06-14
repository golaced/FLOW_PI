#include "include.h"
#include "usart.h"
#include "filter.h"
#include "stmflash.h"
float dt_mems,dt_imu,dt_fushion,dt_uart;

//设置PB9的状态 STM32-MINI开发板的LED控制管脚
void SetPB9(u8 sta)
{		
		static u8 StartFlag = 1;
		if(StartFlag)
		{
			GPIO_InitTypeDef GPIO_InitStructure;
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  	
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			StartFlag = 0;
		}	
		if(sta)	
			GPIO_SetBits(GPIOB, GPIO_Pin_5);	
		else
			GPIO_ResetBits(GPIOB, GPIO_Pin_5);	
}


float acc_flt[3];
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
   	MYDMA_Enable(DMA1_Channel2,TEXT_LENTH); 
 
    SetPB9(0); 
		//READ_PARM();
		//等待中断
		while(1)
		{
		dt_mems=(float)Get_Cycle_T(0)/1000000.;	
			
			
		MPU6050_Read(); 															
	  MPU6050_Data_Prepare( dt_mems );			
		
			
		//imu
    if(cnt[0]++>1){cnt[0]=0;		
		dt_imu=(float)Get_Cycle_T(1)/1000000.;		
		IMUupdate(0.5f *dt_imu,mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, 
			mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z,
	    0,0,0,
			&Roll,&Pitch,&Yaw);
		}
	  //fushion
		if(cnt[1]++>2){cnt[1]=0;		
		dt_fushion=(float)Get_Cycle_T(2)/1000000.;		
		
			
		static float a_br[3]={0};	
		static float acc_temp[3]={0};
		a_br[0] =(float) mpu6050_fc.Acc.x/4096.;//16438.;
		a_br[1] =(float) mpu6050_fc.Acc.y/4096.;//16438.;
		a_br[2] =(float) mpu6050_fc.Acc.z/4096.;//16438.;
		// acc
		acc_temp[0] = a_br[1]*reference_v.z  - a_br[2]*reference_v.y ;
		acc_temp[1] = a_br[2]*reference_v.x  - a_br[0]*reference_v.z ;
	  acc_temp[2] =(reference_v.z *a_br[2] + reference_v.x *a_br[0] + reference_v.y *a_br[1]);
		static float acc_neo_temp[3]={0};
	  acc_neo_temp[0]=-acc_temp[0]*9.87;
		acc_neo_temp[1]=-acc_temp[1]*9.87;
		acc_neo_temp[2]=(acc_temp[2]-1.0f)*9.87;		
		acc_flt[0]=-firstOrderFilter(acc_neo_temp[0],&firstOrderFilters[ACC_LOWPASS_X],dt_fushion);
		acc_flt[1]=firstOrderFilter(acc_neo_temp[1],&firstOrderFilters[ACC_LOWPASS_Y],dt_fushion);
		acc_flt[2]=firstOrderFilter(acc_neo_temp[2],&firstOrderFilters[ACC_LOWPASS_Z],dt_fushion);			
			
		}
		
		//uart
		if(cnt[2]++>3){cnt[2]=0;		
		dt_uart=(float)Get_Cycle_T(3)/1000000.;		
		
					if(DMA_GetFlagStatus(DMA1_FLAG_TC2)!=RESET)//等待通道4传输完成
					{
					DMA_ClearFlag(DMA1_FLAG_TC2);//清除通道4传输完成标志
					SendBuff_cnt=0;

					data_per_uart(
					0,flow_origin[1]*1000,0,
					0,flow_origin[0]*1000,0,
					circle.x,circle.y,circle.z,
					(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),0,0,0/10,0);

					MYDMA_Enable(DMA1_Channel2,SendBuff_cnt);  							
					}	
		}
		
		if(cnt[5]++>25){cnt[5]=0;		
		if(circle.lose_cnt++>10)
		{circle.connect=0;SetPB9(0);}
		}
		Delay_ms(2);
		}
}



