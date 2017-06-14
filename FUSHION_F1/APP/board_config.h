/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：board_config.h
 * 描述     ：系统参数配置     
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H



typedef struct
{
	float kp;
	float kd;
	float ki;
	float kdamp;

}pid_t;

typedef struct 
{
  float x;
	float y;
	float z;
}xyz_f_t;

typedef struct 
{
  int16_t x;
	int16_t y;
	int16_t z;

}xyz_s16_t;


typedef union
{
	char raw_data[64];
	struct
	{
		xyz_f_t Accel;
		xyz_f_t Gyro;
		xyz_f_t Mag;
		xyz_f_t vec_3d_cali;
		unsigned int mpu_flag;
		float Acc_Temperature;
		float Gyro_Temperature;
	}Offset;
}sensor_setup_t; //__attribute__((packed)) 

#endif /* __BOARD_CONFIG_H */
