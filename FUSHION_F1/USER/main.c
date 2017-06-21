#include "include.h"
#include "usart.h"
#include "filter.h"
#include "stmflash.h"
#include "ukf_task.h"
float dt_mems,dt_imu,dt_fushion,dt_uart,dt_sonar;

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
float flow_flt[3];
float k_flow=0.45;
float yaw_qr_off;
float flow_h;	
u8 debug1=0;
u8 end_ble=1;
float accx;//=acc_flt[1]*sin(circle.yaw_off*0.0173)+acc_flt[0]*cos(circle.yaw_off*0.0173);
float accy;//=acc_flt[1]*cos(circle.yaw_off*0.0173)-acc_flt[0]*sin(circle.yaw_off*0.0173);

float flowx;//=flow_flt[1]*sin(circle.yaw_off*0.0173)+flow_flt[0]*cos(circle.yaw_off*0.0173);
float flowy;//=flow_flt[1]*cos(circle.yaw_off*0.0173)-flow_flt[0]*sin(circle.yaw_off*0.0173);	
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
		Delay_ms(100);
		MPU6050_Init(20);
		Delay_ms(100);
		UART_PI_CONFIG(115200);
		UART_FLOW_CONFIG(115200);
		UART_FLOW_CONFIG(576000);
		if(end_ble)
		UART_UP_CONFIG(576000);	
		else
		UART_UP_CONFIG(115200);
		MYDMA_Config(DMA1_Channel2,(u32)&USART3->DR,(u32)SendBuff,TEXT_LENTH);//DMA1通道4,外设为串口1,存储器为SendBuff,长(TEXT_LENTH+2)*100.
	  USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);      
   	MYDMA_Enable(DMA1_Channel2,TEXT_LENTH); 
    Ultrasonic_Init();
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
//		if(circle.check&&circle.connect)
//		yaw_qr_off=circle.yaw-Yaw;
//		else if(circle.connect==0)
//		yaw_qr_off=0;
//	  Yaw+=yaw_qr_off;
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
		acc_flt[1]=-firstOrderFilter(acc_neo_temp[0],&firstOrderFilters[ACC_LOWPASS_Y],dt_fushion);
		acc_flt[0]=firstOrderFilter(acc_neo_temp[1],&firstOrderFilters[ACC_LOWPASS_X],dt_fushion);
		acc_flt[2]=firstOrderFilter(acc_neo_temp[2],&firstOrderFilters[ACC_LOWPASS_Z],dt_fushion);			

		if(ultra_distance<2300)
			flow_h=X_ukf_baro[0];//Moving_Median(0,10,((float)ultra_distance/1000.));
		flow_flt[0]=flow_origin[0]*k_flow*flow_h;
		flow_flt[1]=flow_origin[1]*k_flow*flow_h;
		
	  accx=acc_flt[1]*sin(circle.yaw_off*0.0173)+acc_flt[0]*cos(circle.yaw_off*0.0173);
    accy=acc_flt[1]*cos(circle.yaw_off*0.0173)-acc_flt[0]*sin(circle.yaw_off*0.0173);
	
    flowx=flow_flt[1]*sin(circle.yaw_off*0.0173)+flow_flt[0]*cos(circle.yaw_off*0.0173);
	  flowy=flow_flt[1]*cos(circle.yaw_off*0.0173)-flow_flt[0]*sin(circle.yaw_off*0.0173);	
		
	  ukf_pos_task_qr(0,0,Yaw,flowx,flowy,accx,accy,dt_fushion);
		}
		
		//uart
		if(cnt[2]++>3){cnt[2]=0;		
		dt_uart=(float)Get_Cycle_T(3)/1000000.;		
		
					if(DMA_GetFlagStatus(DMA1_FLAG_TC2)!=RESET)//等待通道4传输完成
					{
					DMA_ClearFlag(DMA1_FLAG_TC2);//清除通道4传输完成标志
					SendBuff_cnt=0;
					if(!debug1){	
          Send_TO_FC();
					Send_TO_FC_OSENSOR();	}
					//mark
					if(cnt[4]++>9&&!debug1){cnt[4]=0;
					Send_TO_FC_OVISON();
					Send_TO_FC_OMARK();	
					}	
					Send_TO_FC_DEBUG();
					if(debug1||end_ble)
					data_per_uart(
					0,flowx*1000,X_ukf[1]*1000,
					X_ukf[3]*100,flowy*1000,X_ukf[4]*1000,
					X_ukf[0]*100, circle.x,-circle.y,
					(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),0,circle.check&&circle.connect,circle.check&&circle.connect,0);
         
					
					MYDMA_Enable(DMA1_Channel2,SendBuff_cnt);  							
					}	
					//uart2
			    Send_TO_FLOW();
		}
		
		if(cnt[3]++>40){cnt[3]=0;	
    dt_sonar=(float)Get_Cycle_T(4)/1000000.;					
		Ultra_Duty();
		}
		
		if(cnt[5]++>25){cnt[5]=0;		
		if(circle.lose_cnt++>10)
		{circle.connect=0;SetPB9(0);}
		}
		Delay_ms(2);
		}
}



