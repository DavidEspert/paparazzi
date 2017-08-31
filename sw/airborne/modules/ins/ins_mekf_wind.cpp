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

// Redifine Eigen assert so it doesn't use memory allocation
#define eigen_assert(_cond) { if (_cond) { while(1) ; } }

// Eigen headers
#pragma GCC diagnostic ignored "-Wshadow"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include "generated/airframe.h"

#include "subsystems/ins.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"

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
	Vector3f accel;
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
static void init_mekf_state(void)
{
  // init state
  mekf_wind_private.state.quat = Quaternionf::Identity();
  mekf_wind_private.state.speed = Vector3f::Zero();
  mekf_wind_private.state.pos = Vector3f::Zero();
  mekf_wind_private.state.rates_bias = Vector3f::Zero();
  mekf_wind_private.state.accel_bias = Vector3f::Zero();
  mekf_wind_private.state.wind = Vector3f::Zero();

  // init measures
  mekf_wind_private.measurements.speed = Vector3f::Zero();
  mekf_wind_private.measurements.pos = Vector3f::Zero();
  mekf_wind_private.measurements.mag = Vector3f::Zero();
  mekf_wind_private.measurements.baro_alt = 0.f;
  mekf_wind_private.measurements.airspeed = 0.f;
  mekf_wind_private.measurements.aoa = 0.f;
  mekf_wind_private.measurements.aos = 0.f;

  // init input
  mekf_wind_private.inputs.rates = Vector3f::Zero();
  mekf_wind_private.inputs.accel = Vector3f::Zero();

  // init state covariance
  mekf_wind_private.P = MEKFWCov::Zero();
  mekf_wind_private.P(0,0) = P0_QUAT;
  mekf_wind_private.P(1,1) = P0_QUAT;
  mekf_wind_private.P(2,2) = P0_QUAT;
  mekf_wind_private.P(3,4) = P0_SPEED;
  mekf_wind_private.P(4,4) = P0_SPEED;
  mekf_wind_private.P(5,5) = P0_SPEED;
  mekf_wind_private.P(6,6) = P0_POS;
  mekf_wind_private.P(7,7) = P0_POS;
  mekf_wind_private.P(8,8) = P0_POS;
  mekf_wind_private.P(9,9) = P0_RATES_BIAS;
  mekf_wind_private.P(10,10) = P0_RATES_BIAS;
  mekf_wind_private.P(11,11) = P0_RATES_BIAS;
  mekf_wind_private.P(12,12) = P0_ACCEL_BIAS;
  mekf_wind_private.P(13,13) = P0_ACCEL_BIAS;
  mekf_wind_private.P(14,14) = P0_ACCEL_BIAS;
  mekf_wind_private.P(15,15) = P0_WIND;
  mekf_wind_private.P(16,16) = P0_WIND;
  mekf_wind_private.P(17,17) = P0_WIND;

  // init process noise
  Matrix<float, MEKF_WIND_PROC_NOISE_SIZE, 1> vp;
  vp(0) = vp(1) = vp(2) = Q_GYRO;
  vp(3) = vp(4) = vp(5) = Q_ACCEL;
  vp(6) = vp(7) = vp(8) = Q_RATES_BIAS;
  vp(9) = vp(10) = vp(11) = Q_ACCEL_BIAS;
  vp(12) = vp(13) = vp(14) = Q_WIND;
  mekf_wind_private.Q = vp.asDiagonal();

  // init measurements noise
  Matrix<float, MEKF_WIND_MEAS_NOISE_SIZE, 1> vm;
  vm(0) = vm(1) = vm(2) = R_SPEED;
  vm(3) = vm(4) = vm(5) = R_POS;
  vm(6) = vm(7) = vm(8) = R_MAG;
  vm(9) = R_BARO;
  vm(10) = R_AIRSPEED;
  vm(11) = R_AOA;
  vm(12) = R_AOS;
  mekf_wind_private.R = vm.asDiagonal();

  // reset flags
  ins_mekf_wind.is_aligned = false;
  ins_mekf_wind.reset = false;
  ins_mekf_wind.baro_initialized = false;
  ins_mekf_wind.gps_fix_once = false;
}

// Some quaternion utility functions
static Quaternionf quat_add(const Quaternionf& q1, const Quaternionf& q2) {
  return Quaternionf(q1.w() + q2.w(), q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z());
}

static Quaternionf quat_smul(const Quaternionf& q1, float scal) {
  return Quaternionf(q1.w() * scal, q1.x() * scal, q1.y() * scal, q1.z() * scal);
}

/**
 * Init function
 */
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
  Quaternionf q_tmp;

  mekf_wind_private.inputs.rates = Vector3f(gyro->p, gyro->q, gyro->r);
  mekf_wind_private.inputs.accel = Vector3f(acc->x, acc->y, acc->z);

  const Vector3f gyro_unbiased = mwp.inputs.rates - mwp.state.rates_bias;
  const Vector3f accel_unbiased = mwp.inputs.accel - mwp.state.accel_bias;
  // propagate state
  // q_dot = 1/2 q * (rates - rates_bias)
  q_tmp.w() = 0.f;
  q_tmp.vec() = gyro_unbiased;
  const Quaternionf q_d = quat_smul(mwp.state.quat * q_tmp, 0.5f);
  // speed_d = q * (accel - accel_bias) * q^-1 + g
  q_tmp.vec() = accel_unbiased;
  mwp.state.accel = (mwp.state.quat * q_tmp * mwp.state.quat.inverse()).vec() + gravity;

  // Euler integration

  //mwp.state.quat = (mwp.state.quat + q_d * dt).normalize();
  mwp.state.quat = quat_add(mwp.state.quat, quat_smul(q_d, dt));
  mwp.state.quat.normalize();
  mwp.state.pos = mwp.state.pos + mwp.state.speed * dt;
  mwp.state.speed = mwp.state.speed + mwp.state.accel * dt;

  //// propagate covariance
  const Matrix3f Rq = mwp.state.quat.toRotationMatrix();

  MEKFWCov A; //FIXME
  A.setZero();
  A.block<3,3>(0,9) = Rq;
  A.block<3,3>(3,0) = (-Rq * accel_unbiased).asDiagonal();
  A.block<3,3>(3,12) = -Rq;
  A.block<3,3>(6,3) = Matrix3f::Identity();

  MEKFWCov An; //FIXME
  An.setZero();
  An.block<3,3>(0,0) = Rq;
  An.block<3,3>(3,3) = Rq;
  An.block<3,3>(9,6) = Matrix3f::Identity();
  An.block<3,3>(12,9) = Matrix3f::Identity();
  An.block<3,3>(15,12) = Matrix3f::Identity();

  MEKFWCov At(A), Ant(An);
  At.transposeInPlace();
  Ant.transposeInPlace();
  //mwp.P = mwp.P + (A * mwp.P * At + An * mwp.Q * Ant);

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


void ins_mekf_wind_align(struct FloatRates *gyro_bias, struct FloatQuat *quat)
{
  /* Compute an initial orientation from accel and mag directly as quaternion */
  mwp.state.quat.w() = quat->qi;
  mwp.state.quat.x() = quat->qx;
  mwp.state.quat.y() = quat->qy;
  mwp.state.quat.z() = quat->qz;

  /* use average gyro as initial value for bias */
  mwp.state.rates_bias(0) = gyro_bias->p;
  mwp.state.rates_bias(1) = gyro_bias->q;
  mwp.state.rates_bias(2) = gyro_bias->r;
}

void ins_mekf_wind_update_mag(struct FloatVect3* mag)
{
  // TODO update mag
  (void)mag;

#if LOG_MEKF_WIND
  if (LogFileIsOpen()) {
    PrintLog(pprzLogFile,
        "%.3f magneto %.3f %.3f %.3f\n",
        get_sys_time_float(),
        mag->x, mag->y, mag->z);
  }
#endif
}

void ins_mekf_wind_update_baro(float baro_alt)
{
  mwp.measurements.baro_alt = baro_alt;

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

/**
 * Getter functions
 */
struct NedCoor_f ins_mekf_wind_get_pos_ned(void)
{
  const struct NedCoor_f p = {
    .x = mwp.state.pos(0),
    .y = mwp.state.pos(1),
    .z = mwp.state.pos(2)
  };
  return p;
}

struct NedCoor_f ins_mekf_wind_get_speed_ned(void)
{
  const struct NedCoor_f s = {
    .x = mwp.state.speed(0),
    .y = mwp.state.speed(1),
    .z = mwp.state.speed(2)
  };
  return s;
}

struct NedCoor_f ins_mekf_wind_get_accel_ned(void)
{
  const struct NedCoor_f a = {
    .x = mwp.state.accel(0),
    .y = mwp.state.accel(1),
    .z = mwp.state.accel(2)
  };
  return a;
}

struct FloatQuat ins_mekf_wind_get_quat(void)
{
  const struct FloatQuat q = {
    .qi = mwp.state.quat.w(),
    .qx = mwp.state.quat.x(),
    .qy = mwp.state.quat.y(),
    .qz = mwp.state.quat.z()
  };
  return q;
}

void ins_mekf_wind_set_quat(struct FloatQuat *quat)
{
  mwp.state.quat.w() = quat->qi;
  mwp.state.quat.x() = quat->qx;
  mwp.state.quat.y() = quat->qy;
  mwp.state.quat.z() = quat->qz;
}

struct FloatRates ins_mekf_wind_get_body_rates(void)
{
  const struct FloatRates r = {
    .p = mwp.inputs.rates(0) - mwp.state.rates_bias(0),
    .q = mwp.inputs.rates(1) - mwp.state.rates_bias(1),
    .r = mwp.inputs.rates(2) - mwp.state.rates_bias(2)
  };
  return r;
}

