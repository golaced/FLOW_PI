#include "include.h"

float dt_mems;
int main(void)
{
		//--------------------------- CLK INIT, HSE PLL ----------------------------
		ErrorStatus HSEStartUpStatus;
		//RCC reset
		RCC_DeInit();
		//�����ⲿʱ�� ��ִ�г�ʼ��
		RCC_HSEConfig(RCC_HSE_ON); 
		//�ȴ��ⲿʱ��׼����
		HSEStartUpStatus = RCC_WaitForHSEStartUp();
		//����ʧ�� ������ȴ�
		while(HSEStartUpStatus == ERROR);
		//�����ڲ�����ʱ��
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		//�ⲿʱ��Ϊ8M ���ﱶƵ��72M
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE); 
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);

		//----------------------------- CLOSE HSI ---------------------------
		//�ر��ڲ�ʱ��HSI
		RCC_HSICmd(DISABLE);	
		
		//��PA��ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			
		//�ж����� 2-level interrupt 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		
		//�����ж�
		__enable_irq(); 
		
	  delay_init(72);
		TIM5_Config();
		Cycle_Time_Init();
		MPU6050_Init(20);
		//�ȴ��ж�
		while(1)
		{
		dt_mems=(float)Get_Cycle_T(0)/1000000.;	
		MPU6050_Read(); 															//??mpu6????
	  MPU6050_Data_Prepare( dt_mems );			//mpu6????????
		Delay_ms(2);
		}
}



