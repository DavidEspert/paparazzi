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
 * @file modules/ins/ins_mekf_wind.h
 *
 * Multiplicative Extended Kalman Filter in rotation matrix formulation.
 *
 * Estimate attitude, ground speed, position, gyro bias, accelerometer bias and wind speed.
 */

#ifndef INS_MEKF_WIND_H
#define INS_MEKF_WIND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"

enum MekfWindStatus {
  MEKF_WIND_UNINIT,
  MEKF_WIND_RUNNING
};

///** filter state vector
// */
//struct MekfWindState {
//	struct FloatQuat quat;
//	struct FloatVect3 speed;
//	struct FloatVect3 pos;
//	struct FloatRates rates_bias;
//	struct FloatVect3 accel_bias;
//	struct FloatVect3 wind;
//};
//
//#define MEKF_WIND_STATE_SIZE 19
//
///** filter command vector
// */
//struct MekfWindInputs {
//	struct FloatRates rates;
//	struct FloatVect3 accel;
//};
//
//#define MEKF_WIND_CMD_SIZE 6
//
///** filter measurement vector
// */
//struct MekfWindMeasurements {
//	struct FloatVect3 speed;
//	struct FloatVect3 pos;
//	struct FloatVect3 mag;
//	float baro_alt;
//	float airspeed;
//	float aoa;
//	float aos;
//};
//
//#define MEKF_WIND_MEAS_SIZE 13

/** filter structure
 */
struct InsMekfWind {
  //struct MekfWindState state;
  //struct MekfWindInputs inputs;
  //struct MekfWindMeasurements measurements;
  struct FloatVect3 mag_h;
  struct OrientationReps body_to_imu;
  enum MekfWindStatus status;
  bool is_aligned;
  bool baro_initialized;
  bool gps_fix_once;
  bool reset;
};

extern struct InsMekfWind ins_mekf_wind;

extern void ins_mekf_wind_init(void);
extern void ins_mekf_wind_propagate(struct FloatRates* gyro,
                                    struct FloatVect3* accel, float dt);
extern bool ins_mekf_wind_align(struct FloatRates *lp_gyro,
                                struct FloatVect3 *lp_accel,
                                struct FloatVect3 *lp_mag);
extern void ins_mekf_wind_set_body_to_imu_quat(struct FloatQuat *quat);
extern void ins_mekf_wind_update_mag(struct FloatVect3* mag);
extern void ins_mekf_wind_update_baro(float pressure);
extern void ins_mekf_wind_update_pos_speed(struct FloatVect3 *pos,
                                           struct FloatVect3 *speed);
extern void ins_mekf_wind_update_airspeed(float airspeed);
extern void ins_mekf_wind_update_incidence(float aoa, float aos);

#ifdef __cplusplus
}
#endif

#endif /* INS_MEKF_WIND_H */

