#include "include.h"
#include "usart.h"
#include "filter.h"
#include "stmflash.h"
#include "ukf_task.h"
#include "iic_vl53.h"
#include "flash.h"
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
#if USE_MINI_FC_FLOW_BOARD
float k_flow=0.3;//0.3
#else
float k_flow=0.45;
#endif
float yaw_qr_off;
float flow_h;	
char debug1=0;
u8 end_ble=1;
float accx;//=acc_flt[1]*sin(circle.yaw_off*0.0173)+acc_flt[0]*cos(circle.yaw_off*0.0173);
float accy;//=acc_flt[1]*cos(circle.yaw_off*0.0173)-acc_flt[0]*sin(circle.yaw_off*0.0173);

float flowx;//=flow_flt[1]*sin(circle.yaw_off*0.0173)+flow_flt[0]*cos(circle.yaw_off*0.0173);
float flowy;//=flow_flt[1]*cos(circle.yaw_off*0.0173)-flow_flt[0]*sin(circle.yaw_off*0.0173);	

float xgyro;
float ygyro;
float Yaw_r;	

u8 up_sel=0;
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
		#if !USE_MINI_FC_FLOW_BOARD
		MPU6050_Init(20);
		Delay_ms(100);
		W25QXX_Init();			//W25QXX初始化
		while(W25QXX_ReadID()!=W25Q32&&W25QXX_ReadID()!=W25Q16)								//检测不到W25Q128
		{	
			Delay_ms(100);
		}
	  READ_PARM();
		#endif
		UART_PI_CONFIG(115200);
		UART_FLOW_CONFIG(115200);
		UART_FLOW_CONFIG(576000);
		if(end_ble)//up link
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
			
		#if !USE_MINI_FC_FLOW_BOARD	
		MPU6050_Read(); 															
	  MPU6050_Data_Prepare( dt_mems );			
		#endif

		//imu
    if(cnt[0]++>1){cnt[0]=0;		
		dt_imu=(float)Get_Cycle_T(1)/1000000.;
    #if !USE_MINI_FC_FLOW_BOARD			
		IMUupdate(0.5f *dt_imu,mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, 
			mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z,
	    0,0,0,
			&Roll,&Pitch,&Yaw_r);
		#endif
		}
		static u8 Yaw_close=0;
		if(Yaw_fc>0)
			Yaw_close=1;
		if(!Yaw_close)
			Yaw=Yaw_r;
		else
			Yaw=Yaw_fc;
//		if(circle.check&&circle.connect)
//		yaw_qr_off=circle.yaw-Yaw;
//		else if(circle.connect==0)
//		yaw_qr_off=0;
//	  Yaw+=yaw_qr_off;
	  //fushion
		if(cnt[1]++>0){cnt[1]=0;		
		dt_fushion=(float)Get_Cycle_T(2)/1000000.;		
		
			
		static float a_br[3]={0};	
		static float acc_temp[3]={0};
		#if !USE_MINI_FC_FLOW_BOARD
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
    #endif		
    if(ultra_distance<2300||sonic_fc!=0)
			flow_h=X_ukf_baro[0];//Moving_Median(0,10,((float)ultra_distance/1000.));
		flow_flt[0]=flow_origin[0]*k_flow*flow_h;
		flow_flt[1]=flow_origin[1]*k_flow*flow_h;
		//旋转安装角度
		#if USE_MINI_FC_FLOW_BOARD
		accx=acc_flt[0]=acc_body[0];
		accy=acc_flt[1]=acc_body[1];
		acc_flt[2]=acc_body[2];
		flowx=flow_flt[1];
		flowy=-flow_flt[0];
		#else
	  accx=acc_flt[1]*sin(circle.yaw_off*0.0173)+acc_flt[0]*cos(circle.yaw_off*0.0173);
    accy=acc_flt[1]*cos(circle.yaw_off*0.0173)-acc_flt[0]*sin(circle.yaw_off*0.0173);
	
    flowx=flow_flt[1]*sin(circle.yaw_off*0.0173)+flow_flt[0]*cos(circle.yaw_off*0.0173);
	  flowy=flow_flt[1]*cos(circle.yaw_off*0.0173)-flow_flt[0]*sin(circle.yaw_off*0.0173);	
		#endif
		xgyro=flow_integrated_ygyro*sin(circle.yaw_off*0.0173)+flow_integrated_xgyro*cos(circle.yaw_off*0.0173);
	  ygyro=flow_integrated_ygyro*cos(circle.yaw_off*0.0173)-flow_integrated_xgyro*sin(circle.yaw_off*0.0173);	
		
	  ukf_pos_task_qr(0,0,Yaw,flowx,flowy,accx,accy,dt_fushion);
		}
		debug1=1;
		//uart
		if(cnt[2]++>2){cnt[2]=0;		
		dt_uart=(float)Get_Cycle_T(3)/1000000.;		
		      
					if(DMA_GetFlagStatus(DMA1_FLAG_TC2)!=RESET)//等待通道4传输完成
					{
					DMA_ClearFlag(DMA1_FLAG_TC2);//清除通道4传输完成标志
					SendBuff_cnt=0;
					#if !USE_MINI_FC_FLOW_BOARD
					if(!debug1){
          #else
			    if(1){
          #endif						
          Send_TO_FC();
					Send_TO_FC_OSENSOR();	
					}
					//mark
					if(cnt[4]++>9&&!debug1){cnt[4]=0;
					Send_TO_FC_OVISON();
					Send_TO_FC_OMARK();	
					}	
					Send_TO_FC_DEBUG();
					if(debug1){
						switch(up_sel){	
						case 0:
						data_per_uart(
						0,X_ukf[1]*100,accy*100,
						0,X_ukf[4]*100,accx*100,
						flowy*100,flowx*100,0,
						(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),0,circle.check&&circle.connect,circle.check&&circle.connect,0);
						break;
						case 1:						
						data_per_uart(
						integrated_y*100,flow_integrated_ygyro*100,accy*100,
						integrated_x*100,flow_integrated_xgyro*100,accx*100,
						flowy*100,flowx*100,0,
						(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),0,circle.check&&circle.connect,circle.check&&circle.connect,0);
					
						break;
						}	
				  }		MYDMA_Enable(DMA1_Channel2,SendBuff_cnt);  					
					}	
					//uart2
			    Send_TO_FLOW();
		}
		static u8 sonar_off=0;
		if(sonic_fc>0)
			sonar_off=1;
		#if USE_SONAR	
		if(cnt[3]++>40){
		#else
		if(cnt[3]++>5){
    #endif			
		cnt[3]=0;	
    dt_sonar=(float)Get_Cycle_T(4)/1000000.;					
		if(sonar_off==0){	
		#if USE_SONAR	
		Ultra_Duty();
		#else
		READ_VL53();
		ultra_distance=ls53.distance;	
		ultra_ok = 1;	
		#endif
		}
		}
		
		if(cnt[5]++>25){cnt[5]=0;		
		if(circle.lose_cnt++>10)
		{circle.connect=0;SetPB9(0);}
		}
		Delay_ms(2);
		}
}



