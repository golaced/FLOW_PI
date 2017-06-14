//pi pa9t pa10r   flow pa2t pa3r  up pb10t pb11r
#include "include.h"


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
void Data_Receive_Anl1(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//??sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//????

  if(*(data_buf+2)==0x03)//RC_PWM
  { 

	}
	
}


void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		
		 
     } 
} 

void Data_Receive_Anl2(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//??sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//????

  if(*(data_buf+2)==0x03)//RC_PWM
  { 

	}
	
}
u8 Rx_Buf[256];	//??????
u8 RxBuffer[50];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
u8 com_data ;
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
					if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
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
	
void USART3_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART3);	//读取接收到的数据
		
		 
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

void data_per_uart(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
u16 i=0; 	
unsigned int temp=0xaF+9;
char ctemp;	
	
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

