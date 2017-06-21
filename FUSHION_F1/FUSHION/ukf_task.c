#include "ukf_task.h"
#include "KF_OLDX_NAV.h"
#include "mymath.h"
#include "usart.h"


int ekf_hml_flag[3]={1,1,1};
float angle_ins[3];




//For  Qr  mark
#define NAV_USE_KF 1
#if USE_FLOW_FLY_ROBOT
float q_flow[3]={0.005,0.005,0.005};//{1,1,1};//0.6;///1;
#else
float q_flow[3]={0.02,0.01,0.01};//{1,1,1};//0.6;///1;
#endif
float r_flow[3]={10,1,0.1};//{1,1,1};//0.6;///1;
float flow_gain=1;//2;
double X_ukf[6];
double X_ukf_baro[6];
int acc_flag_flow[2]={1,1};
float X_ukf_Pos[2];
float r1,r2;
float posNorth,posEast;
float local_Lat,local_Lon;
float velEast,velNorth;
float GPS_J_F,GPS_W_F;
static void CalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

static void CalcGlobalDistance(double lat, double lon) {
    posNorth = (lat - local_Lat) * r1;
    posEast =  (lon - local_Lon) * r2;
}


static void CalcGlobalLocation(posNorth,posEast){ 
    GPS_W_F=(float)posNorth/(float)(r1+0.1)+local_Lat;
    GPS_J_F=(float)posEast/(float)(r2+0.1)+local_Lon;
}


u8 OLDX_KF2(float *measure,float tau,float *r_sensor,u8 *flag_sensor,double *state,double *state_correct,float T)
{
float PosDealt;	
float SpeedDealt;
float K_ACC_Z;
float K_VEL_Z;
float K_POS_Z;

if(!flag_sensor[0]&&!flag_sensor[1]&&!flag_sensor[2])	
	return 0;
K_ACC_Z =(5.0f / (tau * tau * tau));
K_VEL_Z =(3.0f / (tau * tau));
K_POS_Z =(3.0f / tau);
//d spd	
if(flag_sensor[0]&&!flag_sensor[1])	
PosDealt=(measure[0]-state[0]);
else if(!flag_sensor[0]&&flag_sensor[1])
PosDealt=(measure[1]-state[1]);
else if(flag_sensor[1]&&flag_sensor[1])
PosDealt=(measure[0]-state[0])*0.8+(measure[1]-state[1])*(1-0.8);
else 
return 0;	

state_correct[3*0+2] += r_sensor[0]*PosDealt* K_ACC_Z ;
state_correct[3*0+1] += r_sensor[1]*PosDealt* K_VEL_Z ;
state_correct[3*0+0] += r_sensor[2]*PosDealt* K_POS_Z ;

//acc correct
if(flag_sensor[2])	
state[2]=measure[2]+state_correct[0*3+2];
else
return 0;
//else if(flag_sensor[1]&&flag_sensor[2])	
//state[2]=measure[1]+(measure[2]+state_correct[0*3+2]);
	
//d acc
SpeedDealt=state[2]*T;

//pos correct
state_correct[1*3+0]+=(state[1]+0.5*SpeedDealt)*T;
state[0]=state_correct[1*3+0]+state_correct[0*3+0];

//vel correct
state_correct[0*3+1]+=SpeedDealt;
state[1]=state_correct[1*3+1]+state_correct[0*3+1];

return 1;	
}


u8 kf_data_sel=2;//0->flow 1->gps 2->flow global 3->openpilot
double X_KF_NAV[3][3],X_KF1_NAV_X[3],X_KF1_NAV_Y[3],X_KF1_NAV_Z[3];
double P_KF_NAV[3][9];
float ga_nav= 0.1; 
float gwa_nav=0.1;
float g_pos_flow= 0.0086;//0.0051;
float g_spd_flow= 0.0005;//2.00000011e-005;//0.0006;

float K_pos_qr=0.01;
float K_spd_flow=0.86;//1.2;//0.86;
//gps

float K_acc_gps=1;  
float K_pos_gps=1;
float K_spd_gps=1;
float g_pos_gps= 10;
float g_spd_gps= 0.1;//0.1;                          
float velNorth_gps,velEast_gps;
int flag_kf1[2]={1,1};
u8 force_test;
float Posx,Posy;
float r_sensor_flow[4]={0.03,0.05,0.02,2.68};
float r_sensor_hight[4]={0.015,0.05,0.02,4};
void ukf_pos_task_qr(float Qr_x,float Qr_y,float Yaw,float flowx,float flowy,float accx,float accy,float T)
{
	
static double state_correct_x[6],state_correct_y[6],state_correct_z[6];	
static u8 gps_init,kf_init;
static float qr_z_off;
float Sdpx,Accx;
float Sdpy,Accy;
u8 gps_data_vaild=0;
u8 pos_vaild=0;
static u16 cnt_init_kf;	
if(cnt_init_kf++>128&&!kf_init)
	kf_init=1;

if(!kf_init){	
state_correct_x[0]=state_correct_x[1]=state_correct_x[2]=0;
state_correct_y[0]=state_correct_y[1]=state_correct_y[2]=0;	
}
double A[9]=
			 {1,       0,    0,
				T,       1,    0,
				-T*T/2, -T,    1};

double B[3]={T*T/2,T,0}; 
double H[9]={
			 1,0,0,
       0,1,0,
       0,0,0}; 
double H1[9]={
			 1,0,0,
       0,0,0,
       0,0,0}; 

u8 kf_data_sel_temp=kf_data_sel; 
// kf_data_sel_temp=0;
	 float Posz;
	 if(ultra_distance<4000){
	 if(circle.check&&circle.connect)		
   {
	 qr_z_off=(float)circle.z/100.-(float)ultra_distance/1000.;
	 Posz=(float)circle.z/100.-qr_z_off; 
	 }else		 
	 Posz=(float)ultra_distance/1000.;
   }else if(circle.check&&circle.connect)		
	 Posz=(float)circle.z/100.-qr_z_off;
	 float Accz=acc_flt[2];	 
   float Zz[3]={LIMIT(Posz,0.05,10)+LIMIT(X_ukf_baro[1]*T*10,-2,2),0,Accz};
	 u8 flag_hight[3]={1,0,1};
	 if(fabs(X_ukf_baro[0]-Posz)>0.5&&ultra_distance<4000)
	 {X_KF1_NAV_Z[0]=state_correct_z[0]=Posz;X_KF1_NAV_Z[1]=X_KF1_NAV_Z[2]=state_correct_z[1]=state_correct_z[2]=state_correct_z[3]=state_correct_z[4]=state_correct_z[5]=0;}
	 //OLDX_KF2(Zz,r_sensor_hight[3], r_sensor_hight, flag_hight,X_KF1_NAV_Z, state_correct_z, T);
   KF_OLDX_NAV( X_KF_NAV[2],  P_KF_NAV[2],  Zz,Accz, A,  B,  H1,  ga_nav,  gwa_nav, 0.05,  g_spd_flow,  T);
   X_ukf_baro[0]=X_KF_NAV[2][0];
	 X_ukf_baro[1]=X_KF_NAV[2][1];
if(kf_data_sel_temp==1){
 static int qr_yaw_init;	
	 float Yaw_qr=To_180_degrees(Yaw);
	 float ACCY=accy;
   float ACCX=accx;
	 float accEast=ACCY*sin(Yaw_qr*0.0173)+ACCX*cos(Yaw_qr*0.0173);
   float accNorth=ACCY*cos(Yaw_qr*0.0173)-ACCX*sin(Yaw_qr*0.0173);
   float SPDY=flowy*K_spd_flow;
	 float SPDX=flowx*K_spd_flow;
	 float acc_bias[2]={0};

	 velNorth=SPDY*cos(Yaw_qr*0.0173)-SPDX*sin(Yaw_qr*0.0173);
   velEast=SPDY*sin(Yaw_qr*0.0173)+SPDX*cos(Yaw_qr*0.0173);
	 if(circle.check==0&&circle.connect)
	 H[0]=0; 
	 static float pos_reg[2];
   Qr_y=-circle.y;
	 Posy=Qr_y*K_pos_qr;
	 Sdpy=velNorth*K_spd_gps;
	 Accy=accNorth*flag_kf1[1];
	 Qr_x=circle.x;
	 Posx=Qr_x*K_pos_qr;
	 Sdpx=velEast*K_spd_gps;
	 Accx=accEast*flag_kf1[0];
	 static u8 state_init_flow_pos;
	 switch(state_init_flow_pos)
	 {
		 case 0:
			  if(circle.check&&circle.connect)
				{
				state_init_flow_pos=1;
				X_KF_NAV[1][0]=Posy;X_KF_NAV[0][0]=Posx;
				}
			 break;
		 case 1:
			 if(ultra_distance<0.15)
				state_init_flow_pos=0;
			break; 
	 }
	 float Zy[3]={Posy,Sdpy,Accy};
   float Zx[3]={Posx,Sdpx,Accx};

	 u8 flag[3]={0,1,1};
	 OLDX_KF2(Zx,r_sensor_flow[3], r_sensor_flow, flag,X_KF1_NAV_X, state_correct_x, T);
   OLDX_KF2(Zy,r_sensor_flow[3], r_sensor_flow, flag,X_KF1_NAV_Y, state_correct_y, T);
	 X_KF_NAV[0][0]=X_KF1_NAV_X[0];X_KF_NAV[0][1]=X_KF1_NAV_X[1];
	 X_KF_NAV[1][0]=X_KF1_NAV_Y[0];X_KF_NAV[1][1]=X_KF1_NAV_Y[1];

   X_ukf[0]=X_KF_NAV[0][0]+X_ukf[2]*T*10;//East pos
	 //X_ukf[1]=X_KF_NAV[0][1];//East vel
	 X_ukf[2]=X_KF_NAV[0][2];
	 X_ukf[3]=X_KF_NAV[1][0]+X_ukf[5]*T*10;//North  pos
	 //X_ukf[4]=X_KF_NAV[1][1];//North  vel
	 X_ukf[5]=X_KF_NAV[1][2];							
	 X_ukf[1]=-X_KF_NAV[1][1]*sin(Yaw_qr*0.0173)+X_KF_NAV[0][1]*cos(Yaw_qr*0.0173);//X
	 X_ukf[4]= X_KF_NAV[1][1]*cos(Yaw_qr*0.0173)+X_KF_NAV[0][1]*sin(Yaw_qr*0.0173);//Y
   if(fabs( X_ukf[0]-pos_reg[0])>1.5||fabs( X_ukf[3]-pos_reg[1])>1.5){
		  if(circle.connect)
			{X_KF_NAV[1][0]=Posy;X_KF_NAV[0][0]=Posx;}
			else
			{X_KF_NAV[1][0]= pos_reg[1];X_KF_NAV[0][0]= pos_reg[0];}	
		}
	 pos_reg[0]=X_ukf[0];
	 pos_reg[1]=X_ukf[3];	
 	 X_ukf_Pos[0]=X_ukf[0];//East Pos
   X_ukf_Pos[1]=X_ukf[3];//North Pos

}	
else if(kf_data_sel_temp==2){//---------------------flow in global---------------------------------------------------------
	 static int qr_yaw_init;	
	 float Yaw_qr=To_180_degrees(Yaw);
	 float ACCY=accy;
   float ACCX=accx;
	 float accEast=ACCY*sin(Yaw_qr*0.0173)+ACCX*cos(Yaw_qr*0.0173);
   float accNorth=ACCY*cos(Yaw_qr*0.0173)-ACCX*sin(Yaw_qr*0.0173);
   float SPDY=flowy*K_spd_flow;
	 float SPDX=flowx*K_spd_flow;
	 float acc_bias[2]={0};
	 //H[8]=1;  //no acc bias
//	 if(par[0]!=0)g_pos_flow=(float)par[0]/1000.;
//   if(par[1]!=0)
//		 g_spd_flow=(float)par[1]/1000.;
//	 if(par[2]!=0)K_spd_flow=(float)par[2]/1000.;
	 velNorth=SPDY*cos(Yaw_qr*0.0173)-SPDX*sin(Yaw_qr*0.0173);
   velEast=SPDY*sin(Yaw_qr*0.0173)+SPDX*cos(Yaw_qr*0.0173);
	 if(!circle.check&&circle.connect)
	 H[0]=0; 
	 static float pos_reg[2];
   Qr_y=-circle.y;
	 Posy=Qr_y*K_pos_qr;
	 Sdpy=velNorth*K_spd_gps;
	 Accy=accNorth*flag_kf1[1];
	 Qr_x=circle.x;
	 Posx=Qr_x*K_pos_qr;
	 Sdpx=velEast*K_spd_gps;
	 Accx=accEast*flag_kf1[0];
	 static u8 state_init_flow_pos;
	 switch(state_init_flow_pos)
	 {
		 case 0:
			  if(circle.check&&circle.connect)
				{
				state_init_flow_pos=1;
				X_KF_NAV[1][0]=Posy;X_KF_NAV[0][0]=Posx;
				}
			 break;
		 case 1:
			 if(ultra_distance<0.15)
				state_init_flow_pos=0;
			break; 
	 }
	 float Zy[3]={Posy,Sdpy,acc_bias[1]};
	 if(1)//bei 
   KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 float Zx[3]={Posx,Sdpx,acc_bias[0]};
	 if(1)//dong
   KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 X_ukf[0]=X_KF_NAV[0][0]+X_ukf[2]*T*0;//East pos
	 //X_ukf[1]=X_KF_NAV[0][1];//East vel
	 X_ukf[2]=X_KF_NAV[0][2];
	 X_ukf[3]=X_KF_NAV[1][0]+X_ukf[5]*T*0;//North  pos
	 //X_ukf[4]=X_KF_NAV[1][1];//North  vel
	 X_ukf[5]=X_KF_NAV[1][2];							
	 X_ukf[1]=-X_KF_NAV[1][1]*sin(Yaw_qr*0.0173)+X_KF_NAV[0][1]*cos(Yaw_qr*0.0173);//X
	 X_ukf[4]= X_KF_NAV[1][1]*cos(Yaw_qr*0.0173)+X_KF_NAV[0][1]*sin(Yaw_qr*0.0173);//Y
   if(fabs( X_ukf[0]-pos_reg[0])>1.5||fabs( X_ukf[3]-pos_reg[1])>1.5){
		  if(circle.connect)
			{X_KF_NAV[1][0]=Posy;X_KF_NAV[0][0]=Posx;}
			else
			{X_KF_NAV[1][0]= pos_reg[1];X_KF_NAV[0][0]= pos_reg[0];}	
		}
	 pos_reg[0]=X_ukf[0];
	 pos_reg[1]=X_ukf[3];	
 	 X_ukf_Pos[0]=X_ukf[0];//East Pos
   X_ukf_Pos[1]=X_ukf[3];//North Pos
	}else{//--------------------------------------flow in body-------------------------------------------------------------------------	
   if(circle.check==0)
	 H[0]=0; 
   Qr_y=-circle.y;
	 Qr_x=circle.x;  
   Posx=Qr_x*K_pos_qr;
	 Sdpx=flowx*K_spd_flow;
	 Accx=accx*acc_flag_flow[0];
	 float Zx[3]={Posx,Sdpx,0};
   KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 Posy=Qr_y*K_pos_qr;
   Sdpy=flowy*K_spd_flow;
	 Accy=accy*acc_flag_flow[1];
	 float Zy[3]={Posy,Sdpy,0};
   KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 X_ukf[0]=X_KF_NAV[0][0];
	 X_ukf[1]=X_KF_NAV[0][1];
	 X_ukf[2]=X_KF_NAV[0][2];
	 X_ukf[3]=X_KF_NAV[1][0];
	 X_ukf[4]=X_KF_NAV[1][1];
	 X_ukf[5]=X_KF_NAV[1][2];
	 
 	 X_ukf_Pos[0]=X_ukf[0];//X
   X_ukf_Pos[1]=X_ukf[3];//Y
 }

}



