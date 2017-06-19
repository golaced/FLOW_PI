//pi pa9t pa10r   flow pa2t pa3r  up pb10t pb11r
#include "include.h"
#include "ukf_task.h"

void UART_PI_CONFIG(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

void UART_FLOW_CONFIG(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2, ENABLE);                    //使能串口1 

}

void UART_UP_CONFIG(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART1，GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能USART1，GPIOA时钟
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART3, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口1 

}



//------------------------------------------------------------------
float flow_origin_pi[2];
CIRCLE circle;
float mark_map[10][5];//x y z yaw id
float off_yaw=90;
void Data_Receive_Anl1(u8 *data_buf,u8 num)
{ static u8 led;
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//??sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//????
  if(*(data_buf+2)==0x21)//QR
  {
	SetPB9(led);
  led=!led;		
	circle.connect=1;
	circle.lose_cnt=0;
	circle.check=(*(data_buf+4));///10.;
	circle.x=(int16_t)((*(data_buf+5)<<8)|*(data_buf+6));
	circle.y=(int16_t)((*(data_buf+7)<<8)|*(data_buf+8));
	circle.z=(int16_t)((*(data_buf+9)<<8)|*(data_buf+10));
	circle.pit=(int16_t)((*(data_buf+11)<<8)|*(data_buf+12));
	circle.rol=(int16_t)((*(data_buf+13)<<8)|*(data_buf+14));
	circle.yaw=To_180_degrees((int16_t)((*(data_buf+15)<<8)|*(data_buf+16))-off_yaw-circle.yaw_off);	
	flow_origin_pi[0]=circle.spdx=(int16_t)((*(data_buf+17)<<8)|*(data_buf+18));
	flow_origin_pi[1]=circle.spdy=(int16_t)((*(data_buf+19)<<8)|*(data_buf+20));
	//map	
	mark_map[0][0]=(int16_t)((*(data_buf+21)<<8)|*(data_buf+22));
	mark_map[0][1]=(int16_t)((*(data_buf+23)<<8)|*(data_buf+24));
	mark_map[0][2]=(int16_t)((*(data_buf+25)<<8)|*(data_buf+26));
	mark_map[0][3]=(int16_t)((*(data_buf+27)<<8)|*(data_buf+28));
	mark_map[0][4]=*(data_buf+29);
	mark_map[1][0]=(int16_t)((*(data_buf+30)<<8)|*(data_buf+31));
	mark_map[1][1]=(int16_t)((*(data_buf+32)<<8)|*(data_buf+33));
	mark_map[1][2]=(int16_t)((*(data_buf+34)<<8)|*(data_buf+35));
	mark_map[1][3]=(int16_t)((*(data_buf+36)<<8)|*(data_buf+37));
	mark_map[1][4]=*(data_buf+38);
	}	
	else if(*(data_buf+2)==0x22)//QR MAP2
	{
	mark_map[2][0]=(int16_t)((*(data_buf+4)<<8)|*(data_buf+5));
	mark_map[2][1]=(int16_t)((*(data_buf+6)<<8)|*(data_buf+7));
	mark_map[2][2]=(int16_t)((*(data_buf+8)<<8)|*(data_buf+9));
	mark_map[2][3]=(int16_t)((*(data_buf+10)<<8)|*(data_buf+11));
	mark_map[2][4]=*(data_buf+12);
	mark_map[3][0]=(int16_t)((*(data_buf+13)<<8)|*(data_buf+14));
	mark_map[3][1]=(int16_t)((*(data_buf+15)<<8)|*(data_buf+16));
	mark_map[3][2]=(int16_t)((*(data_buf+17)<<8)|*(data_buf+18));
	mark_map[3][3]=(int16_t)((*(data_buf+19)<<8)|*(data_buf+20));
	mark_map[3][4]=*(data_buf+21);
	mark_map[4][0]=(int16_t)((*(data_buf+22)<<8)|*(data_buf+23));
	mark_map[4][1]=(int16_t)((*(data_buf+24)<<8)|*(data_buf+25));
	mark_map[4][2]=(int16_t)((*(data_buf+26)<<8)|*(data_buf+27));
	mark_map[4][3]=(int16_t)((*(data_buf+28)<<8)|*(data_buf+29));
	mark_map[4][4]=*(data_buf+30);	
	mark_map[5][0]=(int16_t)((*(data_buf+31)<<8)|*(data_buf+32));
	mark_map[5][1]=(int16_t)((*(data_buf+33)<<8)|*(data_buf+34));
	mark_map[5][2]=(int16_t)((*(data_buf+35)<<8)|*(data_buf+36));
	mark_map[5][3]=(int16_t)((*(data_buf+37)<<8)|*(data_buf+38));
	mark_map[5][4]=*(data_buf+39);
	}
		
}



u8 RxBuffer1[50];
u8 RxState1 = 0;
u8 RxBufferNum1 = 0;
u8 RxBufferCnt1 = 0;
u8 RxLen1 = 0;
static u8 _data_len1 = 0,_data_cnt1 = 0;
void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
		
	u8 com_data;
		if(USART1->SR & USART_SR_ORE)//ORE??
	{
		com_data = USART1->DR;
	}
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		com_data =USART_ReceiveData(USART1);	//读取接收到的数据
		if(RxState1==0&&com_data==0xAA)
		{
			RxState1=1;
			RxBuffer1[0]=com_data;
		}
		else if(RxState1==1&&com_data==0xAF)
		{
			RxState1=2;
			RxBuffer1[1]=com_data;
		}
		else if(RxState1==2&&com_data>0&&com_data<0XF1)
		{
			RxState1=3;
			RxBuffer1[2]=com_data;
		}
		else if(RxState1==3&&com_data<50)//MAX_send num==50
		{
			RxState1 = 4;
			RxBuffer1[3]=com_data;
			_data_len1 = com_data;
			_data_cnt1 = 0;
		}
		else if(RxState1==4&&_data_len1>0)
		{
			_data_len1--;
			RxBuffer1[4+_data_cnt1++]=com_data;
			if(_data_len1==0)
				RxState1 = 5;
		}
		else if(RxState1==5)
		{
			RxState1 = 0;
			RxBuffer1[4+_data_cnt1]=com_data;
			Data_Receive_Anl1(RxBuffer1,_data_cnt1+5);
		}
		else
			RxState1 = 0;
		 
     } 
} 
	
	

float flow_origin[2];
void Data_Receive_Anl2(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//??sum
	if(!(*(data_buf)==0xCA && *(data_buf+1)==0xCF))		return;		//????

  if(*(data_buf+2)==0x01)//RC_PWM
  { 
   flow_origin[0]=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/1000.;
   flow_origin[1]=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/1000.;
	}
	
}
//flow
u8 RxBuffer[50];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
static u8 _data_len = 0,_data_cnt = 0;
void USART2_IRQHandler(void)                	//串口1中断服务程序
	{
		
	u8 com_data;
		if(USART2->SR & USART_SR_ORE)//ORE??
	{
		com_data = USART2->DR;
	}
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		com_data =USART_ReceiveData(USART2);	//读取接收到的数据
		if(RxState==0&&com_data==0xCA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xCF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)//MAX_send num==50
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_Receive_Anl2(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
		 
     } 
} 
	



float flow_k=1,flow_set_off[3]={0};
void Data_Receive_Anl3(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//??sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//????

  if(*(data_buf+2)==0x66)//
  { 
   flow_k=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/1000.;
   flow_set_off[0]=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/1000.;
	 flow_set_off[1]=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/1000.;
	 circle.yaw_off=flow_set_off[2]=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/10.;	
	}
	
}

u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)                	//串口1中断服务程序
	{
		
	u8 com_data;
		if(USART3->SR & USART_SR_ORE)//ORE??
	{
		com_data = USART3->DR;
	}
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		com_data =USART_ReceiveData(USART3);	//读取接收到的数据
		if(RxState3==0&&com_data==0xAA)
		{
			RxState3=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState3==1&&com_data==0xAF)
		{
			RxState3=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState3==2&&com_data>0&&com_data<0XF1)
		{
			RxState3=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState3==3&&com_data<50)//MAX_send num==50
		{
			RxState3 = 4;
			RxBuffer3[3]=com_data;
			_data_len3 = com_data;
			_data_cnt3 = 0;
		}
		else if(RxState3==4&&_data_len3>0)
		{
			_data_len3--;
			RxBuffer3[4+_data_cnt3++]=com_data;
			if(_data_len3==0)
				RxState3 = 5;
		}
		else if(RxState3==5)
		{
			RxState3 = 0;
			RxBuffer3[4+_data_cnt3]=com_data;
			Data_Receive_Anl3(RxBuffer3,_data_cnt3+5);
		}
		else
			RxState3 = 0;
		 
     } 
} 


//-------

void UsartSend1(uint8_t ch)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, ch); 
}

static void Send_Data1(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend1(dataToSend[i]);
}



void UsartSend2(uint8_t ch)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); 
}

static void Send_Data2(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend2(dataToSend[i]);
}



void UsartSend3(uint8_t ch)
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}

static void Send_Data3(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend3(dataToSend[i]);
}

//--------
u8 SendBuff[TEXT_LENTH];
u16 SendBuff_cnt;

int debug[20];
void data_per_uart(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
u16 i=0; 	
unsigned int temp=0xaF+9;
char ctemp;	

debug[0]=end_ble;
debug[1]=ax;
debug[2]=ay;
debug[3]=az;
debug[4]=gx;
debug[5]=gy;
debug[6]=gz;
debug[7]=hx;	
debug[8]=hy;	
debug[9]=hz;	
debug[10]=yaw;	
debug[11]=pitch;	
debug[12]=roll;	
debug[13]=alt;	
debug[14]=tempr;		
debug[15]=press;	
debug[16]=IMUpersec;			
if(!end_ble){	
SendBuff[SendBuff_cnt++]=0xa5;
SendBuff[SendBuff_cnt++]=0x5a;
SendBuff[SendBuff_cnt++]=14+8;
SendBuff[SendBuff_cnt++]=0xA2;
if(ax<0)ax=32768-ax;
ctemp=ax>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=ax;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

if(ay<0)ay=32768-ay;
ctemp=ay>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=ay;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

if(az<0)az=32768-az;
ctemp=az>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=az;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

if(gx<0)gx=32768-gx;
ctemp=gx>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=gx;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

if(gy<0)gy=32768-gy;
ctemp=gy>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=gy;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
//-------------------------
if(gz<0)gz=32768-gz;
ctemp=gz>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=gz;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

if(hx<0)hx=32768-hx;
ctemp=hx>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=hx;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

if(hy<0)hy=32768-hy;
ctemp=hy>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=hy;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

if(hz<0)hz=32768-hz;
ctemp=hz>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=hz;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

SendBuff[SendBuff_cnt++]=temp%256;
SendBuff[SendBuff_cnt++]=(0xaa);
//
 temp=0xaF+2+2;
SendBuff[SendBuff_cnt++]=(0xa5);
SendBuff[SendBuff_cnt++]=(0x5a);
SendBuff[SendBuff_cnt++]=(14+4);
SendBuff[SendBuff_cnt++]=(0xA1);


if(yaw<0)yaw=32768-yaw;
ctemp=yaw>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=yaw;							
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

if(pitch<0)pitch=32768-pitch;
ctemp=pitch>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=pitch;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
							 
if(roll<0)roll=32768-roll;
ctemp=roll>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=roll;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

if(alt<0)alt=32768-alt;
ctemp=alt>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;				
ctemp=alt;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

if(tempr<0)tempr=32768-tempr;
ctemp=tempr>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=tempr;
SendBuff[SendBuff_cnt++]=ctemp;	   
temp+=ctemp;

if(press<0)press=32768-press;
ctemp=press>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=press;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

ctemp=IMUpersec>>8;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;
ctemp=IMUpersec;
SendBuff[SendBuff_cnt++]=ctemp;
temp+=ctemp;

SendBuff[SendBuff_cnt++]=(temp%256);
SendBuff[SendBuff_cnt++]=(0xaa);
}
}


void Send_TO_FC(void)
{u8 i;	u8 sum = 0;
	u16 _cnt=SendBuff_cnt;
	vs16 _temp;
	SendBuff[SendBuff_cnt++]=0xAA;
	SendBuff[SendBuff_cnt++]=0xAF;
	SendBuff[SendBuff_cnt++]=0x66;//功能字
	SendBuff[SendBuff_cnt++]=0;//数据量

	_temp = X_ukf[0]*100;//pos
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = X_ukf[3]*100;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = X_ukf_baro[0]*1000;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);

	_temp = X_ukf[1]*1000*flow_k;//spd
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = X_ukf[4]*1000*flow_k;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = X_ukf_baro[1]*1000;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	
	_temp = Pitch*100;//att
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = Roll*100;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = Yaw*100;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);

  _temp = circle.connect;
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = circle.check;
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	
	SendBuff[_cnt+3] = SendBuff_cnt-_cnt-4;

	for( i=_cnt;i<SendBuff_cnt;i++)
		sum += SendBuff[i];
	SendBuff[SendBuff_cnt++] = sum;
	
}


void Send_TO_FC_DEBUG(void)
{u8 i;	u8 sum = 0;
	u16 _cnt=SendBuff_cnt;
	vs16 _temp;
	SendBuff[SendBuff_cnt++]=0xAA;
	SendBuff[SendBuff_cnt++]=0xAF;
	SendBuff[SendBuff_cnt++]=0x11;//功能字
	SendBuff[SendBuff_cnt++]=0;//数据量
 for(i=0;i<19;i++){
	_temp = debug[i];//pos
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
 }
	
	SendBuff[_cnt+3] = SendBuff_cnt-_cnt-4;

	for( i=_cnt;i<SendBuff_cnt;i++)
		sum += SendBuff[i];
	SendBuff[SendBuff_cnt++] = sum;
	
}

void Send_TO_FC_OSENSOR(void)
{u8 i;	u8 sum = 0;
	u16 _cnt=SendBuff_cnt;
	vs16 _temp;
	SendBuff[SendBuff_cnt++]=0xAA;
	SendBuff[SendBuff_cnt++]=0xAF;
	SendBuff[SendBuff_cnt++]=0x77;//功能字
	SendBuff[SendBuff_cnt++]=0;//数据量

	_temp = flow_flt[0]*1000;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = flow_flt[1]*1000;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050_fc.Acc.x;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = mpu6050_fc.Acc.y;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = mpu6050_fc.Acc.z;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = mpu6050_fc.Gyro_deg.x*10;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = mpu6050_fc.Gyro_deg.y*10;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = mpu6050_fc.Gyro_deg.z*10;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	
	_temp = ultra_distance;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	
	SendBuff[_cnt+3] = SendBuff_cnt-_cnt-4;

	for( i=_cnt;i<SendBuff_cnt;i++)
		sum += SendBuff[i];
	SendBuff[SendBuff_cnt++] = sum;
	
}


void Send_TO_FC_OVISON(void)
{u8 i;	u8 sum = 0;
	u16 _cnt=SendBuff_cnt;
	vs16 _temp;
	SendBuff[SendBuff_cnt++]=0xAA;
	SendBuff[SendBuff_cnt++]=0xAF;
	SendBuff[SendBuff_cnt++]=0x88;//功能字
	SendBuff[SendBuff_cnt++]=0;//数据量

	_temp = circle.connect;
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = circle.check;
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	
	_temp = circle.x;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = circle.y;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = circle.z;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	
	_temp = circle.pit;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = circle.rol;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = circle.yaw;
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	SendBuff[_cnt+3] = SendBuff_cnt-_cnt-4;

	for( i=_cnt;i<SendBuff_cnt;i++)
		sum += SendBuff[i];
	SendBuff[SendBuff_cnt++] = sum;
	
}


void Send_TO_FC_OMARK(void)
{u8 i;	u8 sum = 0;
	u16 _cnt=SendBuff_cnt;
	vs16 _temp;
	SendBuff[SendBuff_cnt++]=0xAA;
	SendBuff[SendBuff_cnt++]=0xAF;
	SendBuff[SendBuff_cnt++]=0x99;//功能字
	SendBuff[SendBuff_cnt++]=0;//数据量

	for(i=0;i<6;i++){
	_temp = mark_map[i][0];
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = mark_map[i][1];
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = mark_map[i][2];
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = mark_map[i][3];
	SendBuff[SendBuff_cnt++]=BYTE1(_temp);
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
	_temp = mark_map[i][4];
	SendBuff[SendBuff_cnt++]=BYTE0(_temp);
  }

	for( i=_cnt;i<SendBuff_cnt;i++)
		sum += SendBuff[i];
	SendBuff[SendBuff_cnt++] = sum;
	
}
void Send_TO_FLOW(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u16 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量

	_temp = mpu6050_fc.Gyro_deg.x*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mpu6050_fc.Gyro_deg.y*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mpu6050_fc.Gyro_deg.z*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	
	_temp = flow_set_off[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = flow_set_off[1]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt+3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data2(data_to_send, _cnt);
}

