#include "include.h"
#include "sonar.h"
#include "filter.h"
#include "stm32f10x_exti.h"
float T_sonar;
int ultra_distance;
void SONAR_GPIO_Config(void)
{		
	  EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟


		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  	
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  	
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
		GPIO_Init(GPIOB, &GPIO_InitStructure);
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

 
  	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//使能按键所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;					//子优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}

void GPIO_SET_SONAR(u8 num)
{

GPIO_SetBits(GPIOB,GPIO_Pin_0);	
}

void GPIO_RESET_SONAR(u8 num)
{
GPIO_ResetBits(GPIOB,GPIO_Pin_0);
}

u8 GPIO_READ_SONAR(u8 num)
{u8 temp=0;
temp=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1);
return temp;
}

void Ultrasonic_Init()
{ 
 
	SONAR_GPIO_Config();	ultra_ok = 0;

}

s8 ultra_start_f;
u8 ultra_time;
u8 ultra_ok = 0;
void Ultra_Duty()
{
	u8 temp[3];

	ultra_time++;
	ultra_time = ultra_time%2;
	
	GPIO_SET_SONAR(0);
	Delay_us(20);
	GPIO_RESET_SONAR(0);

	ultra_start_f = 1;
}


u8 state_dj[5]={0,0,0,0,0};
u8 state_dj_rx[5]={0,0,0,0,0};
u8 IO_STATE[5]={0,0,0,0,0};
u8 IO_STATER[5]={0,0,0,0,0};
u32 TEMP_SONAR=340*3000/200;//-------------------------------------超声波时间参数
//外部中断4服务程序
u32 cnt_sample1,now_dj[4],lastUpdate_dj[4];

void EXTI1_IRQHandler(void)
{    
	float temp,temp1,temp2;
		static int ultra_distance_old;
	
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
				{
			IO_STATE[3]=GPIO_READ_SONAR(3);
			 switch(state_dj_rx[3])
			 {
				 case 0:if(IO_STATE[3]==1)
				 { 
					 lastUpdate_dj[3] =  micros();
					 state_dj_rx[3]=1; 
				 }
				 break;
				 case 1:		 
				 now_dj[3] = micros();  //读取时间
				 if( now_dj[3] <lastUpdate_dj[3]){cnt_sample1 =  ((float)( now_dj[3]  + (0xffff- lastUpdate_dj[3])) );}
				 else	{ cnt_sample1 =  ((float)( now_dj[3]  - lastUpdate_dj[3])); }
					 if(IO_STATE[3]==1)
						 state_dj_rx[3]=0; 
					 else if(IO_STATE[3]==0)
				 {  
				temp1=LIMIT(Roll,-45,45);
				temp2=LIMIT(Pitch,-45,45);
				temp=(float)cnt_sample1*1000/(TEMP_SONAR)*cos(temp1*0.017)*cos(temp2*0.017);
				temp=((temp)<(0)?(0):((temp)>(4500)?(4500):(temp)));
				ultra_distance=insert_sonar_value_and_get_mode_value(temp);//Moving_Median(1,5,temp);
				ultra_start_f = 0;
				state_dj_rx[3]=0;
		    ultra_ok = 1;
				T_sonar=Get_Cycle_T(4);
				//ultra_delta = (ultra_distance - ultra_distance_old)/LIMIT(T_sonar,0.000000001,1);
				//sonar_filter((float) temp/1000,T_sonar);
				//ultra_distance_old = ultra_distance;
				 }
				
				 break;
				 default:
					 state_dj_rx[3]=0;
					 break;
			 }
				
					EXTI_ClearITPendingBit(EXTI_Line1);
	}   
}

