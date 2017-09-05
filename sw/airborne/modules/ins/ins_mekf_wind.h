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
#include "math/pprz_geodetic_float.h"

// Init functions
extern void ins_mekf_wind_init(void);
extern void ins_mekf_wind_align(struct FloatRates *gyro_bias,
                                struct FloatQuat *quat);
extern void ins_mekf_wind_set_mag_h(const struct FloatVect3 *mag_h);
extern void ins_mekf_wind_reset(void);

// Filtering functions
extern void ins_mekf_wind_propagate(struct FloatRates* gyro,
                                    struct FloatVect3* accel, float dt);
extern void ins_mekf_wind_update_mag(struct FloatVect3* mag);
extern void ins_mekf_wind_update_baro(float baro_alt);
extern void ins_mekf_wind_update_pos_speed(struct FloatVect3 *pos,
                                           struct FloatVect3 *speed);
extern void ins_mekf_wind_update_airspeed(float airspeed);
extern void ins_mekf_wind_update_incidence(float aoa, float aos);

// Getter/Setter functions
extern struct NedCoor_f ins_mekf_wind_get_pos_ned(void);
extern void ins_mekf_wind_set_pos_ned(struct NedCoor_f *p);
extern struct NedCoor_f ins_mekf_wind_get_speed_ned(void);
extern void ins_mekf_wind_set_speed_ned(struct NedCoor_f *s);
extern struct NedCoor_f ins_mekf_wind_get_accel_ned(void);
extern struct FloatQuat ins_mekf_wind_get_quat(void);
extern void ins_mekf_wind_set_quat(struct FloatQuat *quat);
extern struct FloatRates ins_mekf_wind_get_body_rates(void);
extern struct NedCoor_f ins_mekf_wind_get_wind_ned(void);
extern struct NedCoor_f ins_mekf_wind_get_airspeed_body(void);
extern float ins_mekf_wind_get_airspeed_norm(void);

#ifdef __cplusplus
}
#endif

#endif /* INS_MEKF_WIND_H */

