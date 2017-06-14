/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��board_config.h
 * ����     ��ϵͳ��������     
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
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
