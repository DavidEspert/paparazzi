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


#include "modules/ins/ins_mekf_wind.h"

#include "generated/airframe.h"

//#include "subsystems/ins.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_isa.h"

// Eigen headers
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#define MEKF_WIND_COV_SIZE        18
#define MEKF_WIND_PROC_NOISE_SIZE 15
#define MEKF_WIND_MEAS_NOISE_SIZE 13

typedef Matrix<float, MEKF_WIND_COV_SIZE, MEKF_WIND_COV_SIZE> MEKFWCov;
typedef DiagonalMatrix<float, MEKF_WIND_PROC_NOISE_SIZE> MEKFWPNoise;
typedef DiagonalMatrix<float, MEKF_WIND_MEAS_NOISE_SIZE> MEKFWMNoise;

/** filter state vector
 */
struct MekfWindState {
	Quaternionf quat;
	Vector3f speed;
	Vector3f pos;
	Vector3f rates_bias;
	Vector3f accel_bias;
	Vector3f wind;
};

/** filter command vector
 */
struct MekfWindInputs {
	Vector3f rates;
	Vector3f accel;
};

/** filter measurement vector
 */
struct MekfWindMeasurements {
	Vector3f speed;
	Vector3f pos;
	Vector3f mag;
	float baro_alt;
	float airspeed;
	float aoa;
	float aos;
};

/** private filter structure
 */
struct InsMekfWindPrivate {
  struct MekfWindState state;
  struct MekfWindInputs inputs;
  struct MekfWindMeasurements measurements;

  MEKFWCov P;
  MEKFWPNoise Q;
  MEKFWMNoise R;
};

#define P0_QUAT       1.0f
#define P0_SPEED      1.0f
#define P0_POS        1.0f
#define P0_RATES_BIAS 1.0f
#define P0_ACCEL_BIAS 1.0f
#define P0_WIND       1.0f

#define Q_GYRO        1.0f
#define Q_ACCEL       1.0f
#define Q_RATES_BIAS  1.0f
#define Q_ACCEL_BIAS  1.0f
#define Q_WIND        1.0f

#define R_SPEED       1.0f
#define R_POS         1.0f
#define R_MAG         1.0f
#define R_BARO        1.0f
#define R_AIRSPEED    1.0f
#define R_AOA         1.0f
#define R_AOS         1.0f


#undef PERIODIC_TELEMETRY

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


struct InsMekfWind ins_mekf_wind;
static struct InsMekfWindPrivate mekf_wind_private;
// short name
#define mwp mekf_wind_private

/* earth gravity model */
static const Vector3f gravity( 0.f, 0.f, -9.81f);


/* init state and measurements */
static inline void init_mekf_state(void)
{
  // init state
  mekf_wind_private.state.quat.setIdentity();
  mekf_wind_private.state.speed.setZero();
  mekf_wind_private.state.pos.setZero();
  mekf_wind_private.state.rates_bias.setZero();
  mekf_wind_private.state.accel_bias.setZero();
  mekf_wind_private.state.wind.setZero();

  // init measures
  mekf_wind_private.measurements.speed.setZero();
  mekf_wind_private.measurements.pos.setZero();
  mekf_wind_private.measurements.mag.setZero();
  mekf_wind_private.measurements.baro_alt = 0.f;
  mekf_wind_private.measurements.airspeed = 0.f;
  mekf_wind_private.measurements.aoa = 0.f;
  mekf_wind_private.measurements.aos = 0.f;

  // init input
  mekf_wind_private.inputs.rates.setZero();
  mekf_wind_private.inputs.accel.setZero();

  // init state covariance
  mekf_wind_private.P.diagonal() <<
    P0_QUAT, P0_QUAT, P0_QUAT, P0_QUAT,
    P0_SPEED, P0_SPEED, P0_SPEED,
    P0_POS, P0_POS, P0_POS,
    P0_RATES_BIAS, P0_RATES_BIAS, P0_RATES_BIAS,
    P0_ACCEL_BIAS, P0_ACCEL_BIAS, P0_ACCEL_BIAS,
    P0_WIND, P0_WIND, P0_WIND;

  // init process noise
  mekf_wind_private.Q.diagonal() <<
    Q_GYRO, Q_GYRO, Q_GYRO,
    Q_ACCEL, Q_ACCEL, Q_ACCEL,
    Q_RATES_BIAS, Q_RATES_BIAS, Q_RATES_BIAS,
    Q_ACCEL_BIAS, Q_ACCEL_BIAS, Q_ACCEL_BIAS,
    Q_WIND, Q_WIND, Q_WIND;

  // init measurements noise
  mekf_wind_private.R.diagonal() <<
    R_SPEED, R_SPEED, R_SPEED,
    R_POS, R_POS, R_POS,
    R_MAG, R_MAG, R_MAG,
    R_BARO,
    R_AIRSPEED,
    R_AOA,
    R_AOS;

  // reset flags
  ins_mekf_wind.is_aligned = false;
  ins_mekf_wind.reset = false;
  ins_mekf_wind.baro_initialized = false;
  ins_mekf_wind.gps_fix_once = false;
}

void ins_mekf_wind_init(void)
{

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

  // init local earth magnetic field
  ins_mekf_wind.mag_h.x = INS_H_X;
  ins_mekf_wind.mag_h.y = INS_H_Y;
  ins_mekf_wind.mag_h.z = INS_H_Z;

}


void ins_mekf_wind_propagate(struct FloatRates *gyro, struct FloatVect3 *acc, float dt)
{
  mekf_wind_private.inputs.rates << gyro->p, gyro->q, gyro->r;
  mekf_wind_private.inputs.accel << acc->x, acc->y, acc->z;

  const Vector3f gyro_unbiased = mwp.inputs.rates - mwp.state.rates_bias;
  const Vector3f accel_unbiased = mwp.inputs.accel - mwp.state.accel_bias;
  // propagate state
  const Quaternionf q_d = 0.5f * (mwp.state.quat * gyro_unbiased);
  const Vector3f s_d = mwp.state.quat * accel_unbiased * mwp.state.quat.inverse() + gravity;
  const Vector3f p_d = mwp.state.speed;
  mwp.state.quat = (mwp.state.quat + q_d * dt).normalize();
  mwp.state.speed = mwp.state.speed + s_d * dt;
  mwp.state.pos = mwp.state.pos + p_d * dt;

  // propagate covariance
  const Matrix3f Rq = mwp.state.quat.toRotationMatrix();

  MEKFWCov A;
  A.setZero();
  A.block<3,3>(0,9) = Rq;
  A.block<3,3>(3,0) = -(Rq * accel_unbiased);
  A.block<3,3>(3,12) = -Rq;
  A.block<3,3>(6,3) = Matrix3f::Identity();

  MEKFWCov An;
  An.setZero();
  An.block<3,3>(0,0) = Rq;
  An.block<3,3>(3,3) = Rq;
  An.block<3,3>(9,6) = Matrix3f::Identity();
  An.block<3,3>(12,9) = Matrix3f::Identity();
  An.block<3,3>(15,12) = Matrix3f::Identity();

  Matrix3f At, Ant;
  At = A.transpose();
  Ant = An.transpose();
  mwp.P = mwp.P + (A * mwp.P * At + An * mwp.Q * Ant);

#if LOG_MEKF_WIND
  if (LogFileIsOpen()) {
    PrintLog(pprzLogFile,
        "%.3f gyro_accel %.3f %.3f %.3f %.3f %.3f %.3f \n",
        get_sys_time_float(),
        gyro->p, gyro->q, gyro->r, accel->x, accel->y, accel->z
        );
  }
#endif

  // TODO update state interface
}


void ins_mekf_wind_align(struct FloatRates *lp_gyro,
    struct FloatVect3 *lp_accel,
    struct FloatVect3 *lp_mag)
{
  /* Compute an initial orientation from accel and mag directly as quaternion */
  ahrs_float_get_quat_from_accel_mag(&ins_mekf_wind.state.quat, lp_accel, lp_mag);

  /* use average gyro as initial value for bias */
  ins_mekf_wind.state.gyro_bias = *lp_gyro;

  // update private state
  set_state_to_private(&init_mekf_state, &mekf_wind_private);

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

void ins_mekf_wind_update_pos_speed(struct FloatVect3 *pos, struct FloatVect3 *speed)
{
  mwp.measurements.pos(0) = pos->x;
  mwp.measurements.pos(1) = pos->y;
  mwp.measurements.pos(2) = pos->z;
  mwp.measurements.speed(0) = speed->x;
  mwp.measurements.speed(1) = speed->y;
  mwp.measurements.speed(2) = speed->z;

#if LOG_MEKFW_FILTER
	if (LogFileIsOpen()) {
		PrintLog(pprzLogFile,
					"%.3f gps %.3f %.3f %.3f %.3f %.3f %.3f \n",
					get_sys_time_float(),
					pos->x,
					pos->y,
					pos->z,
					speed->x,
					speed->y,
					speed->z
			);
		}
#endif

}

void ins_mekf_wind_update_airspeed(float airspeed)
{
  mwp.measurements.airspeed = airspeed;

#if LOG_MEKF_WIND
  if (LogFileIsOpen()) {
    PrintLog(pprzLogFile,
        "%.3f airspeed %.3f\n", get_sys_time_float(), airspeed);
  }
#endif
}

void ins_mekf_wind_update_incidence(float aoa, float aos)
{
  mwp.measurements.aoa = aoa;
  mwp.measurements.aos = aos;

#if LOG_MEKF_WIND
  if (LogFileIsOpen()) {
    PrintLog(pprzLogFile,
        "%.3f incidence %.3f %.3f\n", get_sys_time_float(), aoa, aos);
  }
#endif
}






#if 0

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

#endif

