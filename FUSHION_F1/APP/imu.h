#ifndef _IMU_H_
#define	_IMU_H_

#include "stm32f10x.h"
#include "board_config.h"
#include "mymath.h"
#include "math.h"

typedef struct 
{
	xyz_f_t err;
	xyz_f_t err_tmp;
	xyz_f_t err_lpf;
	xyz_f_t err_Int;
	xyz_f_t g;
	
}ref_t;
extern xyz_f_t reference_v,acc_3d_hg;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float hx,float hy,float hz,float *rol,float *pit,float *yaw);
extern float  Roll,Pitch,Yaw;   
extern float  ref_q[4],q_nav[4];


#endif

