/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��delay.c
 * ����     ��delay��ʱ����
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
#include "delay.h"
	 
static u8  fac_us=0;//us��ʱ������
static u16 fac_ms=0;//ms��ʱ������

//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void delay_init(u8 SYSCLK)
*��������:		��ʼ���ӳ�ϵͳ��ʹ��ʱ����������״̬
*******************************************************************************/
void delay_init(u8 SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//bit2���,ѡ���ⲿʱ��  HCLK/8
	fac_us=SYSCLK/8;		    
	fac_ms=(u16)fac_us*1000;
}				
				    
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void delay_ms(u16 nms)
*��������:		���뼶��ʱ  ��ʱnms  nms<=1864 
*******************************************************************************/
void Delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL=0x01 ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
}   

//��ʱnus
//nusΪҪ��ʱ��us��.
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void delay_us(u32 nus)
*��������:		΢�뼶��ʱ  ��ʱnus  nms<=1864 
*******************************************************************************/		    								   
void Delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01 ;      //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}

void Delay(u32 x)
{
    u32 i,j;
	for(i=0;i<x;i++)
	   for(j=0;j<500;j++);
}



volatile float Cycle_T[GET_TIME_NUM][3];

enum
{
	NOW = 0,
	OLD,
	NEW,
};

u32 Get_Cycle_T(u8 item)	
{
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//??????
	Cycle_T[item][NOW] = micros(); //?????
	if(Cycle_T[item][NOW]>Cycle_T[item][OLD])
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//?????(??)
	else
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] +(0xFFFF- Cycle_T[item][OLD] )) );//?????(??)	
	return Cycle_T[item][NEW];
}

void Cycle_Time_Init()
{
	u8 i;
	for(i=0;i<GET_TIME_NUM;i++)
	{
		Get_Cycle_T(i);
	}

}



/**************************ʵ�ֺ���********************************************
*����ԭ��:		
*��������:2ms�ж�һ��,������Ϊ2000		
*******************************************************************************/
void TIM5_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		//������ʱ������ʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		//���ö�ʱ������
		TIM_DeInit(TIM3); 
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 								 	//1ms��ʱ			 
		TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);              
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
//		//�ж�����
//		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2 �����ȼ����ж� 
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	 //��Ӧ���ȼ�0 �߼������Ӧ�ж�
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//		NVIC_Init(&NVIC_InitStructure);	  
//		//���ж�
//		TIM_ClearFlag(TIM3, TIM_FLAG_Update);					  
//		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
		//������ʱ��			 
		TIM_Cmd(TIM3, ENABLE); 
}