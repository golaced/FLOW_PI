#ifndef __USART_H
#define __USART_H
#include "include.h"	


void UART_PI_CONFIG(u32 bound);
void UART_FLOW_CONFIG(u32 bound);
void UART_UP_CONFIG(u32 bound);

#define TEXT_LENTH  100			//TEXT_TO_SEND字符串长度(不包含结束符)
extern u8 SendBuff[TEXT_LENTH];
extern u16 SendBuff_cnt;
void data_per_uart(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);

extern float flow_origin[2];
extern float flow_origin_pi[2];

//---------------------
 typedef struct
{
int x,y,z;
int spdx,spdy,spdz;
int x_flp,y_flp;
u8 check;
u8 connect,lose_cnt;
int control[2];
u16 r;
float control_k_miss,control_k;
float control_yaw;
float forward;
float forward_end_dj_pwm;
u8 dj_fly_line;
float pit,rol,yaw;
u8 use_spd;
float yaw_off;
}CIRCLE;
extern CIRCLE circle;
#endif


