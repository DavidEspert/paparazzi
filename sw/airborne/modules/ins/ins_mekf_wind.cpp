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

#ifndef SITL
// Redifine Eigen assert so it doesn't use memory allocation
#define eigen_assert(_cond) { if (!_cond) { while(1) ; } }
#endif

// Eigen headers
#pragma GCC diagnostic ignored "-Wshadow"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

using namespace Eigen;

#define MEKF_WIND_COV_SIZE        18
#define MEKF_WIND_PROC_NOISE_SIZE 15
#define MEKF_WIND_MEAS_NOISE_SIZE 13

typedef Matrix<float, MEKF_WIND_COV_SIZE, MEKF_WIND_COV_SIZE> MEKFWCov;
typedef Matrix<float, MEKF_WIND_PROC_NOISE_SIZE, MEKF_WIND_PROC_NOISE_SIZE> MEKFWPNoise;
typedef Matrix<float, MEKF_WIND_MEAS_NOISE_SIZE, MEKF_WIND_MEAS_NOISE_SIZE> MEKFWMNoise;

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

  /* earth magnetic model */
  Vector3f mag_h;
};

bool ins_mekf_wind_disable_wind = false;

// Initial covariance parameters
#ifndef P0_QUAT
#define P0_QUAT       0.00761544f
#endif
#ifndef P0_SPEED
#define P0_SPEED      1.E-2f
#endif
#ifndef P0_POS
#define P0_POS        1.E-1f
#endif
#ifndef P0_RATES_BIAS
#define P0_RATES_BIAS 1.E-5f
#endif
#ifndef P0_ACCEL_BIAS
#define P0_ACCEL_BIAS 1.E-5f
#endif
#ifndef P0_WIND
#define P0_WIND       1.E-0f
#endif

// Initial process noise parameters
#ifndef Q_GYRO
#define Q_GYRO        1.E-2f
#endif
#ifndef Q_ACCEL
#define Q_ACCEL       1.E-2f
#endif
#ifndef Q_RATES_BIAS
#define Q_RATES_BIAS  1.E-5f
#endif
#ifndef Q_ACCEL_BIAS
#define Q_ACCEL_BIAS  1.E-5f
#endif
#ifndef Q_WIND
#define Q_WIND        1.E+1f
#endif

// Initial measurements noise parameters
#ifndef R_SPEED
#define R_SPEED       0.1f
#endif
#ifndef R_POS
#define R_POS         1.0f
#endif
#ifndef R_MAG
#define R_MAG         1.f
#endif
#ifndef R_BARO
#define R_BARO        0.1f
#endif
#ifndef R_AIRSPEED
#define R_AIRSPEED    0.1f
#endif
#ifndef R_AOA
#define R_AOA         0.1f
#endif
#ifndef R_AOS
#define R_AOS         0.1f
#endif


static struct InsMekfWindPrivate mekf_wind_private;
// short name
#define mwp mekf_wind_private

/* earth gravity model */
static const Vector3f gravity( 0.f, 0.f, 9.81f);


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
}

// Some matrix and quaternion utility functions
static Quaternionf quat_add(const Quaternionf& q1, const Quaternionf& q2) {
  return Quaternionf(q1.w() + q2.w(), q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z());
}

static Quaternionf quat_smul(const Quaternionf& q1, float scal) {
  return Quaternionf(q1.w() * scal, q1.x() * scal, q1.y() * scal, q1.z() * scal);
}

static Matrix3f skew_sym(const Vector3f& v) {
  Matrix3f m;
  m <<    0, -v(2),  v(1),
       v(2),     0, -v(0),
      -v(1),  v(0),     0;
  return m;
}

/**
 * Init function
 */
#include <iostream>
using namespace std;
void ins_mekf_wind_init(void)
{
  // init state and measurements
  init_mekf_state();

  // init local earth magnetic field
  mekf_wind_private.mag_h = Vector3f(1.0f, 0.f, 0.f);
}

void ins_mekf_wind_set_mag_h(const struct FloatVect3 *mag_h)
{
  // update local earth magnetic field
  mekf_wind_private.mag_h(0) = mag_h->x;
  mekf_wind_private.mag_h(1) = mag_h->y;
  mekf_wind_private.mag_h(2) = mag_h->z;
}

void ins_mekf_wind_reset(void)
{
  init_mekf_state();
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
  // store NED accel
  mwp.state.accel = (mwp.state.quat * q_tmp * mwp.state.quat.inverse()).vec() + gravity;

  // Euler integration

  //mwp.state.quat = (mwp.state.quat + q_d * dt).normalize();
  mwp.state.quat = quat_add(mwp.state.quat, quat_smul(q_d, dt));
  mwp.state.quat.normalize();
  mwp.state.speed = mwp.state.speed + mwp.state.accel * dt;
  mwp.state.pos = mwp.state.pos + mwp.state.speed * dt;

  // propagate covariance
  const Matrix3f Rq = mwp.state.quat.toRotationMatrix();
  const Matrix3f Rqdt = Rq * dt;
  const Matrix3f RqA = skew_sym(Rq * (accel_unbiased));
  const Matrix3f RqAdt = RqA * dt;
  const Matrix3f RqAdt2 = RqAdt * dt;

  MEKFWCov A = MEKFWCov::Identity();
  A.block<3,3>(0,9) = -Rqdt;
  A.block<3,3>(3,0) = -RqAdt;
  A.block<3,3>(3,9) = RqAdt2;
  A.block<3,3>(3,12) = -Rqdt;
  A.block<3,3>(6,0) = -RqAdt2;
  A.block<3,3>(6,3) = Matrix3f::Identity() * dt;
  A.block<3,3>(6,9) = RqAdt2 * dt;
  A.block<3,3>(6,12) = -Rqdt * dt;

  Matrix<float, MEKF_WIND_COV_SIZE, MEKF_WIND_PROC_NOISE_SIZE> An;
  An.setZero();
  An.block<3,3>(0,0) = Rq;
  An.block<3,3>(3,3) = Rq;
  An.block<9,9>(9,6) = Matrix<float,9,9>::Identity();

  MEKFWCov At(A);
  At.transposeInPlace();
  Matrix<float, MEKF_WIND_PROC_NOISE_SIZE, MEKF_WIND_COV_SIZE> Ant;
  Ant = An.transpose();

  mwp.P = A * mwp.P * At + An * mwp.Q * Ant * dt;

  if (ins_mekf_wind_disable_wind) {
    mwp.P.block<3,MEKF_WIND_COV_SIZE>(15,0) = Matrix<float,3,MEKF_WIND_COV_SIZE>::Zero();
    mwp.P.block<MEKF_WIND_COV_SIZE-3,3>(0,15) = Matrix<float,MEKF_WIND_COV_SIZE-3,3>::Zero();
    mwp.P(15,15) = P0_WIND;
    mwp.P(16,16) = P0_WIND;
    mwp.P(17,17) = P0_WIND;
    mwp.state.wind = Vector3f::Zero();
  }

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
  mwp.measurements.mag(0) = mag->x;
  mwp.measurements.mag(1) = mag->y;
  mwp.measurements.mag(2) = mag->z;

  // H and Ht matrices
  const Matrix3f Rqt = mwp.state.quat.toRotationMatrix().transpose();
  Matrix<float, 3, MEKF_WIND_COV_SIZE> H = Matrix<float, 3, MEKF_WIND_COV_SIZE>::Zero();
  H.block<3,3>(0,0) = Rqt * skew_sym(mwp.mag_h);
  Matrix<float, MEKF_WIND_COV_SIZE, 3> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt
  Matrix3f S = H * mwp.P * Ht + mwp.R.block<3,3>(6,6);
  // K = P*Ht*S^-1
  Matrix<float, MEKF_WIND_COV_SIZE, 3> K = mwp.P * Ht * S.inverse();
  // Residual z_m - h(z)
  Vector3f res = mwp.measurements.mag - (Rqt * mwp.mag_h);
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,3>(0,0) * res;
  q_tmp.normalize();
  mwp.state.quat = q_tmp * mwp.state.quat;
  mwp.state.quat.normalize();
  mwp.state.speed       += K.block<3,3>(3,0) * res;
  mwp.state.pos         += K.block<3,3>(6,0) * res;
  mwp.state.rates_bias  += K.block<3,3>(9,0) * res;
  mwp.state.accel_bias  += K.block<3,3>(12,0) * res;
  if (!ins_mekf_wind_disable_wind) {
    mwp.state.wind        += K.block<3,3>(15,0) * res;
  }
  // Update covariance
  mwp.P = (MEKFWCov::Identity() - K * H) * mwp.P;

}

void ins_mekf_wind_update_baro(float baro_alt)
{
  mwp.measurements.baro_alt = baro_alt;

  // H and Ht matrices
  Matrix<float, 1, MEKF_WIND_COV_SIZE> H = Matrix<float, 1, MEKF_WIND_COV_SIZE>::Zero();
  H(0,6) = 1.0f;
  Matrix<float, MEKF_WIND_COV_SIZE, 1> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt -> only pos.z component
  float S = mwp.P(8,8) + mwp.R(9,9);
  // K = P*Ht*S^-1
  Matrix<float, MEKF_WIND_COV_SIZE, 1> K = mwp.P * Ht / S;
  // Residual z_m - h(z)
  float res = mwp.measurements.baro_alt - mwp.state.pos(2);
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,1>(0,0) * res;
  q_tmp.normalize();
  mwp.state.quat = q_tmp * mwp.state.quat;
  mwp.state.quat.normalize();
  mwp.state.speed       += K.block<3,1>(3,0) * res;
  mwp.state.pos         += K.block<3,1>(6,0) * res;
  mwp.state.rates_bias  += K.block<3,1>(9,0) * res;
  mwp.state.accel_bias  += K.block<3,1>(12,0) * res;
  if (!ins_mekf_wind_disable_wind) {
    mwp.state.wind        += K.block<3,1>(15,0) * res;
  }
  // Update covariance
  mwp.P = (MEKFWCov::Identity() - K * H) * mwp.P;
}

void ins_mekf_wind_update_pos_speed(struct FloatVect3 *pos, struct FloatVect3 *speed)
{
  mwp.measurements.pos(0) = pos->x;
  mwp.measurements.pos(1) = pos->y;
  mwp.measurements.pos(2) = pos->z;
  mwp.measurements.speed(0) = speed->x;
  mwp.measurements.speed(1) = speed->y;
  mwp.measurements.speed(2) = speed->z;

  // H and Ht matrices
  Matrix<float, 6, MEKF_WIND_COV_SIZE> H = Matrix<float, 6, MEKF_WIND_COV_SIZE>::Zero();
  H.block<6,6>(0,3) = Matrix<float,6,6>::Identity();
  Matrix<float, MEKF_WIND_COV_SIZE, 6> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt
  Matrix<float, 6, 6> S = mwp.P.block<6,6>(3,3) + mwp.R.block<6,6>(0,0);
  // K = P*Ht*S^-1
  Matrix<float, MEKF_WIND_COV_SIZE, 6> K = mwp.P * Ht * S.inverse();
  // Residual z_m - h(z)
  Matrix<float, 6, 1> res = Matrix<float, 6, 1>::Zero();
  res.block<3,1>(0,0) = mwp.measurements.speed - mwp.state.speed;
  res.block<3,1>(3,0) = mwp.measurements.pos - mwp.state.pos;
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,6>(0,0) * res;
  q_tmp.normalize();
  mwp.state.quat = q_tmp * mwp.state.quat;
  mwp.state.speed       += K.block<3,6>(3,0) * res;
  mwp.state.pos         += K.block<3,6>(6,0) * res;
  mwp.state.rates_bias  += K.block<3,6>(9,0) * res;
  mwp.state.accel_bias  += K.block<3,6>(12,0) * res;
  if (!ins_mekf_wind_disable_wind) {
    mwp.state.wind        += K.block<3,6>(15,0) * res;
  }
  // Update covariance
  mwp.P = (MEKFWCov::Identity() - K * H) * mwp.P;
}

void ins_mekf_wind_update_airspeed(float airspeed)
{
  mwp.measurements.airspeed = airspeed;

  if (ins_mekf_wind_disable_wind) return;
  // H and Ht matrices
  const RowVector3f IuRqt = mwp.state.quat.toRotationMatrix().transpose().block<1,3>(0,0);
  const Vector3f va = mwp.state.speed - mwp.state.wind;
  Matrix<float, 1, MEKF_WIND_COV_SIZE> H = Matrix<float, 1, MEKF_WIND_COV_SIZE>::Zero();
  H.block<1,3>(0,0) = IuRqt * skew_sym(va);
  H.block<1,3>(0,3) = IuRqt;
  H.block<1,3>(0,15) = -IuRqt;
  Matrix<float, MEKF_WIND_COV_SIZE, 1> Ht = H.transpose();
  // S = H*P*Ht + Hn*N*Hnt
  float S = H * mwp.P * Ht + mwp.R(10,10);
  // K = P*Ht*S^-1
  Matrix<float, MEKF_WIND_COV_SIZE, 1> K = mwp.P * Ht / S;
  // Residual z_m - h(z)
  float res = mwp.measurements.airspeed - IuRqt * va;
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,1>(0,0) * res;
  q_tmp.normalize();
  mwp.state.quat = q_tmp * mwp.state.quat;
  mwp.state.quat.normalize();
  mwp.state.speed       += K.block<3,1>(3,0) * res;
  mwp.state.pos         += K.block<3,1>(6,0) * res;
  mwp.state.rates_bias  += K.block<3,1>(9,0) * res;
  mwp.state.accel_bias  += K.block<3,1>(12,0) * res;
  if (!ins_mekf_wind_disable_wind) {
    mwp.state.wind        += K.block<3,1>(15,0) * res;
  }
  // Update covariance
  mwp.P = (MEKFWCov::Identity() - K * H) * mwp.P;
}

void ins_mekf_wind_update_incidence(float aoa, float aos)
{
  mwp.measurements.aoa = aoa;
  mwp.measurements.aos = aos;

  if (ins_mekf_wind_disable_wind) return;
  // H and Ht matrices
  const Matrix3f Rqt = mwp.state.quat.toRotationMatrix().transpose();
  const Vector3f va = Rqt * (mwp.state.speed - mwp.state.wind); // airspeed in body frame
  // check if data in valid range
  const float van = va.norm();
  //const float va_bound = 0.2f * van;
  //cout << van << endl;
  //cout << va << endl;
  //cout << mwp.state.pos(2) << endl;
  if (van < 5.f /*|| va(0) < 0.8f * van
      || va(1) < - va_bound || va(1) > va_bound
      || va(2) < - va_bound || va(2) > va_bound*/
      || mwp.state.pos(2) > -10.f) {
    // filter doesn't work at zero airspeed
    //cout << "out of bound" << endl;
    return;
  }
  //cout << "inside bound" << endl;
  const RowVector3f C(sinf(aoa), 0.f, -cosf(aoa));
  const RowVector3f CRqt = C * Rqt;
  const float s_aos = sinf(aos);
  const float c_aos = cosf(aos);
  const Matrix3f B = Vector3f(s_aos * s_aos, - c_aos * c_aos, 0.f).asDiagonal();
  const RowVector3f vBRqt = 2.f * va.transpose() * B * Rqt;
  Matrix<float, 2, MEKF_WIND_COV_SIZE> H = Matrix<float, 2, MEKF_WIND_COV_SIZE>::Zero();
  H.block<1,3>(0,0) = CRqt * skew_sym(mwp.state.speed - mwp.state.wind);
  H.block<1,3>(0,3) = CRqt;
  H.block<1,3>(0,15) = -CRqt;
  H.block<1,3>(1,0) = vBRqt * skew_sym(mwp.state.speed - mwp.state.wind);
  H.block<1,3>(1,3) = vBRqt;
  H.block<1,3>(1,15) = -vBRqt;
  Matrix<float, MEKF_WIND_COV_SIZE, 2> Ht = H.transpose();
  // Hn and Hnt matrices
  Matrix2f Hn = Matrix2f::Identity();
  Hn(0,0) = C(2) * va(0) - C(0) * va(2);
  const float s_2aos = sinf(2.0f * aos);
  Hn(1,1) = (RowVector3f(-s_2aos, 0.f, s_2aos) * va.asDiagonal()) * va;
  Matrix2f Hnt = Hn.transpose();
  //cout << endl << H << endl;
  //cout << H*mwp.P*Ht << endl;
  //cout << Hn << endl;
  //cout << Hn * mwp.R.block<2,2>(11,11) * Hnt << endl;
  // S = H*P*Ht + Hn*N*Hnt
  Matrix2f S = H * mwp.P * Ht + Hn * mwp.R.block<2,2>(11,11) * Hnt;
  //cout << S << endl;
  //cout << S.inverse() << endl;
  // K = P*Ht*S^-1
  Matrix<float, MEKF_WIND_COV_SIZE, 2> K = mwp.P * Ht * S.inverse();
  // Residual z_m - h(z)
  Vector2f res = Vector2f::Zero();
  res(0) = - C * va;
  res(1) = - va.transpose() * B * va;
  //cout << endl;
  //cout << mwp.state.speed << endl;
  //cout << mwp.state.wind << endl;
  //cout << mwp.measurements.aoa << " " << C*va << endl;
  //cout << mwp.measurements.aos << " " << va.transpose() * B * va << endl;
  //cout << va << endl;
  //cout << B << endl;
  //cout << K << endl;
  //cout << res << endl;
  // Update state
  Quaternionf q_tmp;
  q_tmp.w() = 1.f;
  q_tmp.vec() = 0.5f * K.block<3,2>(0,0) * res;
  q_tmp.normalize();
  mwp.state.quat = q_tmp * mwp.state.quat;
  mwp.state.quat.normalize();
  mwp.state.speed       += K.block<3,2>(3,0) * res;
  mwp.state.pos         += K.block<3,2>(6,0) * res;
  mwp.state.rates_bias  += K.block<3,2>(9,0) * res;
  mwp.state.accel_bias  += K.block<3,2>(12,0) * res;
  if (!ins_mekf_wind_disable_wind) {
    mwp.state.wind        += K.block<3,2>(15,0) * res;
  }
  // Update covariance
  mwp.P = (MEKFWCov::Identity() - K * H) * mwp.P;
}

/**
 * Getter/Setter functions
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

void ins_mekf_wind_set_pos_ned(struct NedCoor_f *p)
{
  mwp.state.pos(0) = p->x;
  mwp.state.pos(1) = p->y;
  mwp.state.pos(2) = p->z;
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

void ins_mekf_wind_set_speed_ned(struct NedCoor_f *s)
{
  mwp.state.speed(0) = s->x;
  mwp.state.speed(1) = s->y;
  mwp.state.speed(2) = s->z;
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

struct NedCoor_f ins_mekf_wind_get_wind_ned(void)
{
  const struct NedCoor_f w = {
    .x = mwp.state.wind(0),
    .y = mwp.state.wind(1),
    .z = mwp.state.wind(2)
  };
  return w;
}

struct NedCoor_f ins_mekf_wind_get_airspeed_body(void)
{
  const Matrix3f Rqt = mwp.state.quat.toRotationMatrix().transpose();
  const Vector3f va = Rqt * (mwp.state.speed - mwp.state.wind); // airspeed in body frame
  const struct NedCoor_f a = {
    .x = va(0),
    .y = va(1),
    .z = va(2)
  };
  return a;
}

float ins_mekf_wind_get_airspeed_norm(void)
{
  return (mwp.state.speed - mwp.state.wind).norm();
}

struct FloatVect3 ins_mekf_wind_get_accel_bias(void)
{
  const struct FloatVect3 ab = {
    .x = mwp.state.accel_bias(0),
    .y = mwp.state.accel_bias(1),
    .z = mwp.state.accel_bias(2)
  };
  return ab;
}

struct FloatRates ins_mekf_wind_get_rates_bias(void)
{
  const struct FloatRates rb = {
    .p = mwp.state.rates_bias(0),
    .q = mwp.state.rates_bias(1),
    .r = mwp.state.rates_bias(2)
  };
  return rb;
}

