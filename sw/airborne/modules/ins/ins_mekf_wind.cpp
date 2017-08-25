/*
 * Copyright (C) 2017 Marton Brossard <martin.brossard@mines-paristech.fr>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ins/ins_mekf_wind.cpp
 *
 * Multiplicative Extended Kalman Filter in rotation matrix formulation.
 *
 * Estimate attitude, ground speed, position, gyro bias, accelerometer bias and wind speed.
 *
 * Using Eigen library
 */


#include "mekf/ins/ins_mekf_wind.h"

#include "generated/airframe.h"

#include "subsystems/ins.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

#define MEKF_WIND_USE_UTM TRUE
#if MEKF_WIND_USE_UTM
#include "firmwares/fixedwing/nav.h"
#endif

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_isa.h"


#if SEND_MEKF_WIND_FILTER || PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif


#if LOG_MEKF_WIND
#ifndef SITL
#include "modules/loggers/sdlog_chibios.h"
#define PrintLog sdLogWriteLog
#define LogFileIsOpen() (pprzLogFile != -1)
#else // SITL: print in a file
#include <stdio.h>
#define PrintLog fprintf
#define LogFileIsOpen() (pprzLogFile != NULL)
static FILE* pprzLogFile = NULL;
#endif
#endif

static const struct FloatMat33 id33 = {{1, 0 ,0},
  {0, 1, 0},
  {0, 0, 1}
};

static const struct FloatMat33 zero33 = {{0, 0 ,0},
  {0, 0, 0},
  {0, 0, 0}
};

static const float id1818[18][18] = {{ 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.},
  {  0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. },
  {  0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. },
  {  0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. },
  {  0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. },
  {  0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. },
  {  0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. },
  {  0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. },
  {  0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0. },
  {  0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0. },
  {  0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0. },
  {  0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0. },
  {  0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0. },
  {  0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0. },
  {  0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0. },
  {  0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0. },
  {  0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0. },
  {  0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1. },
};

static const float id_u[3] = {1, 0, 0};
static const float id_z[3] = {0, 0, 1};

struct InsMekfWind ins_mekf_wind;

/* earth gravity model */
static const struct FloatVect3 g = { 0.f, 0.f, -9.81f };


/* init state and measurements */
static inline void init_mekf_state(void)
{
  // init state
  float_quat_identity(&ins_mekf_wind.state.quat);
  FLOAT_VECT3_ZERO(ins_mekf_wind.state.speed);
  FLOAT_VECT3_ZERO(ins_mekf_wind.state.pos);
  FLOAT_RATES_ZERO(ins_mekf_wind.state.rates_bias);
  FLOAT_VECT3_ZERO(ins_mekf_wind.state.accel_bias);
  FLOAT_VECT3_ZERO(ins_mekf_wind.state.wind);
  FLOAT_VECT3_ZERO(ins_mekf_wind.state.airspeed);

  // init measures
  FLOAT_VECT3_ZERO(ins_mekf_wind.measurement.y_gpsspeed);
  FLOAT_VECT3_ZERO(ins_mekf_wind.measurement.y_gpspos);
  FLOAT_VECT3_ZERO(ins_mekf_wind.measurement.y_mag);
  ins_mekf_wind.measurement.baro = 0.f;
  ins_mekf_wind.measurement.airspeed = 0.f;
  ins_mekf_wind.measurement.aoa = 0.f;
  ins_mekf_wind.measurement.aos = 0.f;

  // init input
  FLOAT_RATES_ZERO(ins_mekf_wind.inputs.rates);
  FLOAT_VECT3_ZERO(ins_mekf_wind.inputs.accel);

  // init state covariance
  float P0[18][18] = {{P0diag[1],0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,P0diag[2].,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,P0diag[3].,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,P0diag[4].,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,P0diag[5],0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,P0diag[6].,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,P0diag[7].,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,P0diag[8].,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,P0diag[9].,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,P0diag[10].,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,P0diag[11].,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,P0diag[12].,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,P0diag[13].,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,P0diag[14],0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,P0diag[15],0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,P0diag[16],0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,P0diag[17],0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,P0diag[18]},
  };
  memcpy(ins_mekf_wind.P, P0, sizeof(P0));

  float Q0[15][15] = {{Q0diag[1],0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,Q0diag[2].,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,Q0diag[3].,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,Q0diag[4].,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,Q0diag[5],0.,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,Q0diag[6].,0.,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,Q0diag[7].,0.,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,Q0diag[8].,0.,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,Q0diag[9].,0.,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,Q0diag[10].,0.,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,Q0diag[11].,0.,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,Q0diag[12].,0.,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,Q0diag[13].,0.,0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,Q0diag[14],0.},
    {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,Q0diag[15]},
  };
  memcpy(ins_mekf_wind.Q, Q0, sizeof(Q0));

  ins_mekf_wind.R_diagonale = {NOISE_GPSSPEED_X, NOISE_GPSSPEED_Y, NOISE_GPSSPEED_Z,
    NOISE_GPSPOS_X, NOISE_GPSPOS_Y, NOISE_GPSPOS_Z,
    AHRS_MAG_NOISE_X, AHRS_MAG_NOISE_Y, AHRS_MAG_NOISE_Z,
    NOISE_BARO,
    NOISE_PITOT,
    NOISE_AOA,
    NOISE_AOS};

  // init magnetometers
  ins_mekf_wind.b.x = INS_H_X;
  ins_mekf_wind.b.y = INS_H_Y;
  ins_mekf_wind.b.z = INS_H_Z;

  ins_mekf_wind.is_aligned = false;
  ins_mekf_wind.reset = false;
}

void ins_mekf_wind_init(void)
{
  // init position
#if MEKF_WIND_USE_UTM
  struct UtmCoor_f utm0;
  utm0.north = (float)nav_utm_north0;
  utm0.east = (float)nav_utm_east0;
  utm0.alt = GROUND_ALT;
  utm0.zone = nav_utm_zone0;
  stateSetLocalUtmOrigin_f(&utm0);
  stateSetPositionUtm_f(&utm0);
#else
  struct LlaCoor_i llh_nav0;
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);
  struct LtpDef_i ltp_def;
  ltp_def_from_ecef_i(&ltp_def, &ecef_nav0);
  ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ltp_def);
#endif

#if LOG_MEKF_WIND && SITL
  // open log file for writing
  // path should be specified in airframe file
  uint32_t counter = 0;
  char filename[512];
  snprintf(filename, 512, "%s/mekf_wind_%05d.csv", STRINGIFY(MEKF_WIND_LOG_PATH), counter);
  // check availale name
  while ((pprzLogFile = fopen(filename, "r"))) {
    fclose(pprzLogFile);
    snprintf(filename, 512, "%s/mekf_wind_%05d.csv", STRINGIFY(MEKF_WIND_LOG_PATH), ++counter);
  }
  pprzLogFile = fopen(filename, "w");
  if (pprzLogFile == NULL) {
    printf("Failed to open WE log file '%s'\n",filename);
  } else {
    printf("Opening WE log file '%s'\n",filename);
  }
#endif

  // init state and measurements
  init_mekf_state();
}


void ins_mekf_wind_propagate(struct FloatRates *gyro, struct FloatVect3 *acc, float dt)
{
  propagate_state(gyro, acc, dt);
  propagate_covariance(gyro, acc, dt);

#if LOG_MEKF_WIND
  if (LogFileIsOpen()) {
    PrintLog(pprzLogFile,
        "%.3f gyro_accel %.3f %.3f %.3f %.3f %.3f %.3f \n",
        get_sys_time_float(),
        gyro->p, gyro->q, gyro->r, accel->x, accel->y, accel->z
        );
  }
#endif
}


void ins_mekf_wind_align(struct FloatRates *lp_gyro,
    struct FloatVect3 *lp_accel,
    struct FloatVect3 *lp_mag)
{
  /* Compute an initial orientation from accel and mag directly as quaternion */
  ahrs_float_get_quat_from_accel_mag(&ins_mekf_wind.state.quat, lp_accel, lp_mag);

  /* use average gyro as initial value for bias */
  ins_mekf_wind.state.gyro_bias = *lp_gyro;

  // ins and ahrs are now running
  ins_mekf_wind.is_aligned = true;
}

void ins_mekf_wind_update_mag(struct FloatVect3* mag)
{
  struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ins_mekf_wind.body_to_imu);
  // new values in body frame
  float_rmat_transp_vmult(&ins_mekf_wind.measurements.mag, body_to_imu_rmat, mag);

#if LOG_MEKF_WIND
  if (LogFileIsOpen()) {
    PrintLog(pprzLogFile,
        "%.3f magneto %.3f %.3f %.3f\n",
        get_sys_time_float(),
        mag->x, mag->y, mag->z);
  }
#endif
}

void ins_mekf_wind_set_body_to_imu_quat(struct FloatQuat *quat)
{
}

void ins_mekf_wind_update_baro(float pressure)
{
  static float ins_qfe = 101325.0f;
  static float alpha = 10.0f;
  static int32_t i = 1;
  static float baro_moy = 0.0f;
  static float baro_prev = 0.0f;

  if (!ins_mekf_wind.baro_initialized) {
    // try to find a stable qfe
    // TODO generic function in pprz_isa ?
    if (i == 1) {
      baro_moy = pressure;
      baro_prev = pressure;
    }
    baro_moy = (baro_moy * (i - 1) + pressure) / i;
    alpha = (10.*alpha + (baro_moy - baro_prev)) / (11.0f);
    baro_prev = baro_moy;
    // test stop condition
    if (fabs(alpha) < 0.005f) {
      ins_qfe = baro_moy;
      ins_mekf_wind.baro_initialized = true;
    }
    if (i == 250) {
      ins_qfe = pressure;
      ins_mekf_wind.baro_initialized = true;
    }
    i++;
  } else { /* normal update with baro measurement */
    ins_mekf_wind.measurements.baro_alt = -pprz_isa_height_of_pressure(pressure, ins_qfe); // Z down
  }

#if LOG_MEKF_WIND
  if (LogFileIsOpen()) {
    PrintLog(pprzLogFile,
        "%.3f  baro %.3f \n",
        get_sys_time_float(),
        ins_mekf_wind.measurements.baro_alt);
  }
#endif
}

void ins_mekf_wind_update_gps(struct GpsState *gps_s)
{
	if (gps_s->fix >= GPS_FIX_3D) {
    ins_mekf_wind.gps_fix_once = true;

#if MEKF_WIND_USE_UTM
		if (state.utm_initialized_f) {
			struct UtmCoor_f utm = utm_float_from_gps(gps_s, nav_utm_zone0);
			// position (local ned)
			ins_mekf_wind.measurements.pos.x = utm.north - state.utm_origin_f.north;
			ins_mekf_wind.measurements.pos.y = utm.east - state.utm_origin_f.east;
			ins_mekf_wind.measurements.pos.z = state.utm_origin_f.alt - utm.alt;
			// speed
			ins_mekf_wind.measurements.speed.x = gps_s->ned_vel.x / 100.0f;
			ins_mekf_wind.measurements.speed.y = gps_s->ned_vel.y / 100.0f;
			ins_mekf_wind.measurements.speed.z = gps_s->ned_vel.z / 100.0f;
		}

#else
		if (state.ned_initialized_f) {
			struct NedCoor_i gps_pos_cm_ned, ned_pos;
			ned_of_ecef_point_i(&gps_pos_cm_ned, &state.ned_origin_i, &gps_s->ecef_pos);
			INT32_VECT3_SCALE_2(ned_pos, gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
			NED_FLOAT_OF_BFP(ins_mekf_wind.measurements.pos, ned_pos);
			struct EcefCoor_f ecef_vel;
			ECEF_FLOAT_OF_BFP(ecef_vel, gps_s->ecef_vel);
			ned_of_ecef_vect_f(&ins_mekf_wind.measurements.speed, &state.ned_origin_f, &ecef_vel);
		}
#endif
	}

#if LOG_MEKFW_FILTER
	if (LogFileIsOpen()) {
		PrintLog(pprzLogFile,
					"%.3f gps %.3f %.3f %.3f %.3f %.3f %.3f \n",
					get_sys_time_float(),
					ins_mekf_wind.measurements.pos.x,
					ins_mekf_wind.measurements.pos.y,
					ins_mekf_wind.measurements.pos.z,
					ins_mekf_wind.measurements.speed.x,
					ins_mekf_wind.measurements.speed.y,
					ins_mekf_wind.measurements.speed.z
			);
		}
#endif

}

void ins_mekf_wind_update_airspeed(float airspeed)
{
#if LOG_MEKF_WIND
  if (LogFileIsOpen()) {
    PrintLog(pprzLogFile,
        "%.3f airspeed %.3f\n", get_sys_time_float(), airspeed);
  }
#endif
}

void ins_mekf_wind_update_incidence(float aoa, float aos)
{
#if LOG_MEKF_WIND
  if (LogFileIsOpen()) {
    PrintLog(pprzLogFile,
        "%.3f incidence %.3f %.3f\n", get_sys_time_float(), aoa, aos);
  }
#endif
}








static inline void propagate_state(struct FloatRates *omega, struct FloatVect3 *acc, float dt)
{
	struct FloatRates omega_unbiased = *omega;
	struct FloatVect3 *acc_unbiased = *acc;
	/* unbias measurements */
	RATES_SUB(omega_unbiased, ins_mekf_wind.state.omega_b);
	VECT3_SUB(acc_unbiased, ins_mekf_wind.state.acc_b);

	/* propagate rotation matrix */
	struct FloatMat33 omega_skew;
	SKEW_VECT3(omega_skew, omega_unbiased);
	MAT_MUL(3, 3, 3, ins_mekf_wind.state.rot, ins_mekf_wind.state.rot, omega_skew);

	/* propagate speed */
	struct FloatVect3 tem = g;
	struct FloatVect3 tem2;
	MAT33_VECT3_MUL(tem2, ins_mekf_wind.state.rot, acc_unbiased);
	VECT3_ADD(tem, tem2);
	VECT3_SUM_SCALED(ins_mekf_wind.state.v, ins_mekf_wind.state.v, tem, dt);

	/* propagate position */
	struct FloatVect3 tem;
	MAT33_VECT3_MUL(tem, ins_mekf_wind.state.rot, ins_mekf_wind.state.v);
	VECT3_SUM_SCALED(ins_mekf_wind.state.x, ins_mekf_wind.state.x, tem, dt);

	/* propagate biases and wind */
	// assumed firstly random walk
	
	UPDATE_VA(ins_mekf_wind.state.va, ins_mekf_wind.state.rot, ins_mekf_wind.state.v, ins_mekf_wind.state.w);

}


static inline void propagate_covariance(struct FloatRates *omega, struct FloatVect3 *acc, float dt)// optimisable
{  
	struct FloatMat33 tem;
	struct FloatMat33 tem2;
	struct FloatVect3 *acc_unbiased = *acc;
	VECT3_SUB(acc_unbiased, ins_mekf_wind.state.acc_b);
	MAT33_VECT3_MUL(tem2, ins_mekf_wind.state.rot, acc_unbiased);
	SKEW_VECT3(tem, tem2);
	float A[18][18] = {{zero33, zero33, zero33, ins_mekf_wind.state.rot, zero33, zero33},
			{tem, zero33, zero33, zero33, -ins_mekf_wind.state.rot, zero33},
			{zero33, id33, zero33, zero33, zero33, zero33},
			{zero33, zero33, zero33, zero33, zero33, zero33},//bias follow random walk
			{zero33, zero33, zero33, zero33, zero33, zero33},
			{zero33, zero33, zero33, zero33, zero33, zero33}};

	//A * P * A'
	float APA[18][18];
	MAT_MUL(18, 18, 18, APA, A, ahrs_mlkf.P);
	MAT_MUL_T(18, 18, 18, APA, ahrs_mlkf.P, A);


	// A_n * Q * A_n'
	float An[18][18] = {{ins_mekf_wind.state.rot, zero33, zero33, zero33, zero33, zero33},
			{zero33, ins_mekf_wind.state.rot, zero33, zero33, zero33, zero33},
			{zero33, zero33, zero33, id33, zero33, zero33},
			{zero33, zero33, zero33, zero33, id33, zero33},
			{zero33, zero33, zero33, zero33, zero33, id33}};
	float Q[18][18];
	DIAG_VEC(18, Q, Q_diag);
	float AQA[18][18];
	MAT_MUL(18, 18, 18, AQA, An, Q);
	MAT_MUL_T(18, 18, 18,  Q, AQA, An);

	// P = P + (A*P*A' + A_n*Q*A_n)*dt
	float tmp[18][18];
	MAT_ADD(18, 18, tmp, APA, AQA);
	SCAL_MUL_MAT(18, 18, tmp, dt);
	MAT_ADD(18, 18, ins_mekf_wind.P, tmp)

}


static inline void update_state(void)
{
	// y_mesured : observations
	// y_times[0] = 1, GPS measurement, 0 otherwise
	// y_times[1] = 1, magnetometer measurement, 0 otherwise
	// y_times[2] = 1, barometer measurement, 0 otherwise	
	// y_times[3] = 1, Pitot tube measurement, 0 otherwise
	// y_times[4] = 1, vanes measurement, 0 otherwise



	//H
	struct FloatMat33 b_skew, rot_transpose, tem, tem2, tem3, tem4, tem5, tem6, tem7, tem8_skew;
	const struct FloatMat33 B = {{sin(y_mesured[12])^2, 0, 0},
			{0, -cos(y_mesured[12]), 0},
			{0, 0, 0}
	};
	SKEW_VECT3(b_skew, ahrs_ins_mekf_wind.b);
	struct FloatVect3 C = {sin(y_mesured[11]), 0 -cos(y_mesured[11])},
			tem8 = ins_mekf_wind.state.v-ins_mekf_wind.state.wind;
	SKEW_VECT3(tem8_skew, tem8);
	MAT33_TRANSPOSE(rot_transpose, ins_mekf_wind.state.rot);
	MAT_MUL(3, 3, 3, tem, rot_transpose, b_skew);
	MAT_MUL(1, 3, 3, tem2, id_u, rot_transpose);
	MAT_MUL(3, 3, 3, tem3, rot_transpose, tem8_skew);
	MAT_MUL(1, 3, 3, tem3, id_u, tem3);
	MAT_MUL(3, 3, 3, tem4, C, rot_transpose);
	MAT_MUL(3, 3, 3, tem, rot_transpose, b_skew);
	MAT_MUL(3, 3, 3, tem5, tem4, tem8);
	MAT_MUL(3, 3, 3, tem6, B, rot_transpose);
	MAT_MUL(1, 3, 3, tem7, ins_mekf_wind.state.airspeed, 2*tem6);
	MAT_MUL(3, 3, 3, tem6, tem6, tem8);
	const float H[6][18] = {{zero33, id33, zero33, zero33, zero33, zero33},
			{zero33, zero33, id33, zero33, zero33, zero33},
			{tem, zero33, zero33, zero33, zero33, zero33},
			{0, 0, id_z, 0, 0, 0},
			{tem3, tem2, 0, 0, 0, -tem2},
			{tem5, tem4, 0, 0, 0, -tem4},
			{tem6, tem7, 0, 0, 0, -tem7}};


	// S = HPH' + JRJ
	int nb_measurements;
	float tmp[13][18];
	MAT_MUL(13, 18, 18, tmp, H, ahrs_ins_mekf_wind.P);
	float S[13][13];
	MAT_MUL_T(13, 18, 13, S, tmp, H);

	/* add the measurement noise */
	for (l=0; l<_i; l++)                                            
		MAT33_ELMT(S,i,i) += ins_mekf_wind.measurementnoise[i];         
	}

	float invS[18][18];
	float u_left[18][18] = ins_mekf_wind.P, diagonale[18], v_right[18][18];
	dsvd( u_left, 13, 13, diagonale, v_right);
	VECT_INV(13, diagonale, diagonale);
	MAT_MUL(13, 13, 1, u_left, u_left,diagonale);
	MAT_MUL(13, 13, 13, invS, u_left,diagonale);

	// K = PH'invS
	float tmp2[18][13];
	MAT_MUL_T(18, 18, 13, tmp2, ins_mekf_wind.P, H);
	float K[18][13];
	MAT_MUL(18, 13, 13, K, tmp2, invS);

	// P = (I-KH)P
	float tmp3[18][18];
	MAT_MUL(18, 13, 18, tmp3, K, H);

	float tmp4[18][18];
	MAT_SUB(18, 18, tmp4, id1818, tmp3);
	float tmp5[18][18];
	MAT_MUL(18, 18, 18, tmp5, tmp4, ins_mekf_wind.P);
	memcpy(ins_mekf_wind.P, tmp5, sizeof(ins_mekf_wind.P));

	// x = x + K*err
	float y_talphahat, y_tbetahat;
	MAT_MUL_VECT(3, y_talphahat, C, ins_mekf_wind.state.va);
	struct FloatVect3 tem;
	MAT_MUL_VECT(3, tem, B, ins_mekf_wind.state.va);
	MAT_MUL(1, 3, 1, y_tbetahat, ins_mekf_wind.state.va, tem);
	float err[18] = {ins_mekf_wind.state.v, ins_mekf_wind.state.x, y_mhat, ins_mekf_wind.state.x[3], ins_mekf_wind.state.va[1], y_talphahat, y_tbetahat];
	int j;
	for (j=0; j<18; j++){                      
			err[j] = err[j] - y_mesured[j];
	}
	//test pour prendre uniquement les mesures
	float inov[18];
	MAT_MUL_VECT(18, inov, K, err);

	struct FloatVect3 tmp;
	VECT3_DIFF(e, *b_measured, b_expected);
	//rot
	tmp = {err[3], err[4], err[5]};
	MAT_SUB(3, 3, ins_mekf_wind.state.v, ins_mekf_wind.state.v, tmp);
	tmp = {err[6], err[7], err[8]};
	MAT_SUB(3, 3, ins_mekf_wind.state.x, ins_mekf_wind.state.x, tmp);
	tmp = {err[9], err[10], err[11]};
	MAT_SUB(3, 3, ins_mekf_wind.state.omega_b, ins_mekf_wind.state.omega_b, tmp);
	tmp = {err[12], err[13], err[14]};
	MAT_SUB(3, 3, ins_mekf_wind.state.a_b, ins_mekf_wind.state.a_b, tmp);
	tmp = {err[15], err[16], err[17]};
	MAT_SUB(3, 3, ins_mekf_wind.state.w, ins_mekf_wind.state.w, tmp);
	COMPUTE_VA(ins_mekf_wind.state.rot, ins_mekf_wind.state.v, ins_mekf_wind.state.x);
}

