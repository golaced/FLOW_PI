#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#define micros() TIM3->CNT


void delay_init(u8 SYSCLK);
void Delay_ms(u16 nms);
void Delay_us(u32 nus);
void Delay(u32 x);
void Initial_System_Timer(void);
#define GET_TIME_NUM 10
u16 Get_Time(u8,u16,u16);
u32 Get_Cycle_T(u8 );
void Cycle_Time_Init(void);
uint32_t GetSysTime_us(void);
void TIM5_Config(void);
#endif

//------------------End of File----------------------------
