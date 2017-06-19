#ifndef _UKF_TASK_H
#define _UKF_TASK_H

#include "stm32f10x.h"
#define NAV_MIN_GPS_ACC		3.0f					    // minimum gps hAcc needed to enter auto nav modes, in meters
#define NAV_MAX_GPS_AGE		1e6					    // maximum age of position update needed to enter auto nav modes, in microseconds
#define NAV_MIN_FIX_ACC		4.0f					    // minimum gps hAcc still considered a valid "2D" fix, in meters
#define NAV_MAX_FIX_AGE		10e6					    // maximum age of position update still considered a valid "2D" fix, in microseconds

#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters
#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))
#define M_PI			3.14159265f
#define M_PI_2			(M_PI / 2.0f)
#define NAV_HF_HOME_DIST_D_MIN	2.0f						// do not compute dynamic bearing when closer than this to home position (zero to never compute)
#define NAV_HF_HOME_DIST_FREQ	4						// update distance to home at this Hz, should be > 0 and <= 400
#define NAV_HF_HOME_BRG_D_MAX	1.0f * DEG_TO_RAD				// re-compute headfree reference angles when bearing to home changes by this many degrees (zero to always re-compute)
#define NAV_HF_DYNAMIC_DELAY	((int)3e6f)					// delay micros before entering dynamic mode after switch it toggled high
#define RAD_TO_DEG		(180.0f / M_PI)
#define DEG_TO_RAD		(M_PI / 180.0f)

extern float K_spd_flow;
extern  double X_ukf[6],X_ukf_nav[9],X_ukf_all[9],X_ukf_baro[6];;
extern double X_KF_NAV[3][3];
extern float X_ukf_Pos[2];
extern float velEast,velNorth, velNorth_gps,velEast_gps;;
void ukf_flow(float flowx,float flowy,float accx,float accy,float T);
void ukf_pos_task(double JIN,double WEI,float Yaw,float flowx,float flowy,float accx,float accy,float T);
void ukf_pos_task_qr(float Qr_x,float Qr_y,float Yaw,float flowx,float flowy,float accx,float accy,float T);


//openpilot

struct mag_sensor {
	uint8_t id[4];
	uint8_t updated;
	struct {
		int16_t axis[3];
	} raw;
	struct {
		float axis[3];
	} scaled;
	struct {
		float bias[3];
		float scale[3];
		float variance[3];
	} calibration;
};

//! Contains the data from the accelerometer
struct accel_sensor {
	struct {
		uint16_t x;
		uint16_t y;
		uint16_t z;
	} raw;
	struct {
		float x;
		float y;
		float z;
	} filtered;
	struct {
		float scale[3][4];
		float variance[3];
	} calibration;
};

//! Contains the data from the gyro
struct gyro_sensor {
	struct {
		uint16_t x;
		uint16_t y;
		uint16_t z;
	} raw;
	struct {
		float x;
		float y;
		float z;
	} filtered;
	struct {
		float bias[3];
		float scale[3];
		float variance[3];
		float tempcompfactor[3];
	} calibration;
	struct {
		uint16_t xy;
		uint16_t z;
	} temp;
};

//! Conains the current estimate of the attitude
struct attitude_solution {
	struct {
		float q1;
		float q2;
		float q3;
		float q4;
	} quaternion;
};

//! Contains data from the altitude sensor
struct altitude_sensor {
	float altitude;
	u8 updated;
};

//! Contains data from the GPS (via the SPI link)
struct gps_sensor {
	float NED[3];
	float heading;
	float groundspeed;
	float quality;
	u8 updated;
};

#endif
