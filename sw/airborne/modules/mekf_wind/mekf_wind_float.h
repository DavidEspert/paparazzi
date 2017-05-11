/*
 * Copyright (C) 2012-2013 Jean-Philippe Condomines, Gautier Hattenberger
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
 * @file modules/mekf_wind/mekf_wind_float.h
 * MEKF-Wind filter.
 * For more information, please send an email to "jp.condomines@gmail.com"
 */

#ifndef MEKFW_FLOAT_H
#define MEKFW_FLOAT_H

#include "subsystems/ins.h"
#include "subsystems/gps.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"


/** filter measurement vector
 */
struct mekfw_measures {
  struct NedCoor_f pos_gps;   ///< Measured gps position
  struct NedCoor_f speed_gps; ///< Measured gps speed
  struct FloatVect3 mag;      ///< Measured magnetic field
  float baro_alt;             ///< Measured barometric altitude
 // float airspeed;             ///< Measured airspeed
 // float aoa;             ///< Measured aoa
 // float aos;             ///< Measured aos
};

/** filter command vector dimension
 */
#define MEKFW_COMMAND_DIM 6

/** filter command vector
 */
struct mekfw_command {
  struct FloatRates rates;  ///< Input gyro rates
  struct FloatVect3 accel;  ///< Input accelerometers
};


/** filter structure
 */
struct MekwFloat {
  struct mekfw_measures meas;           ///< measurement vector
  bool reset;                       ///< flag to request reset/reinit the filter


  struct FloatVect3 mag_h;
  bool is_aligned;
};

extern struct MekwFloat mekf_wind_float;

extern void mekf_wind_float_init(void);
extern void mekf_wind_float_propagate(struct FloatRates* gyro,
                                          struct FloatVect3* accel, float dt);
extern void mekf_wind_float_update_mag(struct FloatVect3* mag);
extern void mekf_wind_float_update_baro(float pressure);
extern void mekf_wind_float_update_gps(struct GpsState *gps_s);

#endif /* MEKFW_FLOAT_H */

