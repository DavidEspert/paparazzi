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
 * @file modules/mekf_wind/mekf_wind_float.c
 * @author Jean-Philippe Condomines <jp.condomines@gmail.com>
 *
 * MEKF-Wind filter.
 *
 */

#include "modules/mekf_wind/mekf_wind_float.h"

#include "subsystems/ahrs/ahrs_int_utils.h"
#include "subsystems/ahrs/ahrs_aligner.h"

#include "subsystems/ins.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"
#if MEKFW_USE_UTM
#include "firmwares/fixedwing/nav.h"
#endif

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_rk_float.h"
#include "math/pprz_isa.h"

#include "state.h"

// for debugging
#if SEND_MEKFW_FILTER || PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif


#if LOG_MEKFW_FILTER
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


struct MekfwFloat mekf_wind_float;

/* barometer */
//bool ins_baro_initialized;
bool mekf_wind_baro_initialized;

/* gps */
//bool ins_gps_fix_once;
bool mekf_wind_gps_fix_once;
/* B magnétique */
//static const struct FloatVect3 B = { (float)(x), (float)(y), (float)(z) };

/* error computation */
//static inline void error_output(struct MekfwFloat *_ins);

/* propagation model (called by runge-kutta library) */
//static inline void mekf_wind_model(float *o, const float *x, const int n, const float *u, const int m);




/* init  measurements */
static inline void init_mekfw_state(void)
{
	// init measures
	FLOAT_VECT3_ZERO(mekf_wind_float.meas.pos_gps);
	FLOAT_VECT3_ZERO(mekf_wind_float.meas.speed_gps);
	//mekf_wind_float.meas.baro = 0.0f;
	//mekf_wind_float.meas.airspeed = 0.0f;
	//mekf_wind_float.meas.aoa = 0.0f; // non nécéssaire
	//mekf_wind_float.meas.aos = 0.0f; // non nécéssaire

	// init baro
	//ins_baro_initialized = false;
        mekf_wind_baro_initialized = false;
	//ins_gps_fix_once = false;
        mekf_wind_gps_fix_once = false;
}



void mekf_wind_float_init(void)
{

	// init position
#if MEKFW_USE_UTM
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
	//stateSetLocalOrigin_i(&ltp_def);
#endif

#if LOG_MEKFW_FILTER && SITL
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
	init_mekfw_state();
}


void mekf_wind_float_propagate(struct FloatRates* gyro __attribute__((unused)), struct FloatVect3* accel __attribute__((unused)), float dt __attribute__((unused)))
{
/*	struct FloatRates body_rates;
	// fill command vector in body frame
	struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&mekf_wind_float.body_to_imu);
	float_rmat_transp_ratemult(&mekf_wind_float.cmd.rates, body_to_imu_rmat, gyro);
	float_rmat_transp_vmult(&mekf_wind_float.cmd.accel, body_to_imu_rmat, accel);

*/
	
#if LOG_MEKFW_FILTER
	if (LogFileIsOpen()) {
		PrintLog(pprzLogFile,
				"%.3f  gyro_accel %.3f %.3f %.3f %.3f %.3f %.3f \n",
				get_sys_time_float(),
				gyro->p,//mekf_wind_float.cmd.rates.p,
				gyro->q,//mekf_wind_float.cmd.rates.q,
				gyro->r,//mekf_wind_float.cmd.rates.r,
				accel->x,//mekf_wind_float.cmd.accel.x,
				accel->y,//mekf_wind_float.cmd.accel.y,
				accel->z//mekf_wind_float.cmd.accel.z,
		);
	}
#endif
}

void mekf_wind_float_update_gps(struct GpsState *gps_s)
{
	if (gps_s->fix >= GPS_FIX_3D) {
		//ins_gps_fix_once = true;
                mekf_wind_gps_fix_once = true;

#if MEKFW_USE_UTM
		if (state.utm_initialized_f) {
			struct UtmCoor_f utm = utm_float_from_gps(gps_s, nav_utm_zone0);
			// position (local ned)
			mekf_wind_float.meas.pos_gps.x = utm.north - state.utm_origin_f.north;
			mekf_wind_float.meas.pos_gps.y = utm.east - state.utm_origin_f.east;
			mekf_wind_float.meas.pos_gps.z = state.utm_origin_f.alt - utm.alt;
			// speed
			mekf_wind_float.meas.speed_gps.x = gps_s->ned_vel.x / 100.0f;
			mekf_wind_float.meas.speed_gps.y = gps_s->ned_vel.y / 100.0f;
			mekf_wind_float.meas.speed_gps.z = gps_s->ned_vel.z / 100.0f;
		}
		//nouveau code pour envoyer sur log
		//fin nouveau  
#else
		if (state.ned_initialized_f) {
			struct NedCoor_i gps_pos_cm_ned, ned_pos;
			ned_of_ecef_point_i(&gps_pos_cm_ned, &state.ned_origin_i, &gps_s->ecef_pos);
			INT32_VECT3_SCALE_2(ned_pos, gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
			NED_FLOAT_OF_BFP(mekf_wind_float.meas.pos_gps, ned_pos);
			struct EcefCoor_f ecef_vel;
			ECEF_FLOAT_OF_BFP(ecef_vel, gps_s->ecef_vel);
			ned_of_ecef_vect_f(&mekf_wind_float.meas.speed_gps, &state.ned_origin_f, &ecef_vel);
		}
#endif
	}
	
#if LOG_MEKFW_FILTER
	if (LogFileIsOpen()) {
		PrintLog(pprzLogFile,
					"%.3f gps %.3f %.3f %.3f %.3f %.3f %.3f \n",
					get_sys_time_float(),
					mekf_wind_float.meas.pos_gps.x,
					mekf_wind_float.meas.pos_gps.y,
					mekf_wind_float.meas.pos_gps.z,
					mekf_wind_float.meas.speed_gps.x,
					mekf_wind_float.meas.speed_gps.y,
					mekf_wind_float.meas.speed_gps.z
			);
		}
#endif

}


void mekf_wind_float_update_baro(float pressure)
{
	static float ins_qfe = 101325.0f;
	static float alpha = 10.0f;
	static int32_t i = 1;
	static float baro_moy = 0.0f;
	static float baro_prev = 0.0f;

	//if (!ins_baro_initialized) {
        if (!mekf_wind_baro_initialized) {
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
			//ins_baro_initialized = true;
                        mekf_wind_baro_initialized = true;
		}
		if (i == 250) {
			ins_qfe = pressure;
			//ins_baro_initialized = true;
                        mekf_wind_baro_initialized = true;
		}
		i++;
	} else { /* normal update with baro measurement */
		mekf_wind_float.meas.baro_alt = -pprz_isa_height_of_pressure(pressure, ins_qfe); // Z down
	}
	//nouveau code pour envoyer sur log
#if LOG_MEKFW_FILTER
	if (LogFileIsOpen()) {
		PrintLog(pprzLogFile,
					"%.3f  baro %.3f \n",
					get_sys_time_float(),
					mekf_wind_float.meas.baro_alt
			);
	}
#endif
	//fin nouveau    
}

// assume mag is dead when values are not moving anymore
#define MAG_FROZEN_COUNT 30

void mekf_wind_float_update_mag(struct FloatVect3* mag __attribute__((unused)))
{
/*	static uint32_t mag_frozen_count = MAG_FROZEN_COUNT;
	static int32_t last_mx = 0;

	if (last_mx == mag->x) {
		mag_frozen_count--;
		if (mag_frozen_count == 0) {
			// if mag is dead, better set measurements to zero
			FLOAT_VECT3_ZERO(mekf_wind_float.meas.mag);
			mag_frozen_count = MAG_FROZEN_COUNT;
		}
	} else {
		// values are moving
		//struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&mekf_wind_float.body_to_imu);
		// new values in body frame
		//float_rmat_transp_vmult(&mekf_wind_float.meas.mag, body_to_imu_rmat, mag);
		// reset counter
		mag_frozen_count = MAG_FROZEN_COUNT;
	}
	last_mx = mag->x;*/
	//nouveau code pour envoyer sur log
#if LOG_MEKFW_FILTER
	if (LogFileIsOpen()) {
		PrintLog(pprzLogFile,
					"%.3f  magneto %.3f %.3f %.3f\n",
					get_sys_time_float(),
					mag->x,//mekf_wind_float.meas.mag.x,
					mag->y,//mekf_wind_float.meas.mag.y,
					mag->z//mekf_wind_float.meas.mag.z,
			);
	}
#endif
	//fin nouveau  	
}

/*
static inline void mekf_wind_model(float *o, const float *x, const int n, const float *u,
		const int m __attribute__((unused)))
{


	struct mekf_wind_command *c = (struct mekf_wind_command *)u;

}
*/




void mekf_wind_float_update_incidence(float aoa __attribute__((unused)), float aos __attribute__((unused))) {
  
  //envoyer log
#if LOG_MEKFW_FILTER
	if (LogFileIsOpen()) {
		PrintLog(pprzLogFile,
                    "%.3f incidence %.3f %.3f\n", get_sys_time_float(), aoa, aos);
	}
#endif

}



void mekf_wind_float_update_airspeed(float airspeed __attribute__((unused)))
{
#if LOG_MEKFW_FILTER
	if (LogFileIsOpen()) {
		PrintLog(pprzLogFile,
                          "%.3f airspeed %.3f\n", get_sys_time_float(), airspeed);
	}  
#endif
}
