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
#include "modules/loggers/sdlog_chibios.h"
bool log_started = false;
#endif

struct MekfwFloat mekf_wind_float;

/* barometer */
bool ins_baro_initialized;

/* gps */
bool ins_gps_fix_once;

/* error computation */
static inline void error_output(struct MekfwFloat *_ins);

/* propagation model (called by runge-kutta library) */
static inline void mekf_wind_model(float *o, const float *x, const int n, const float *u, const int m);




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
	ins_baro_initialized = false;
	ins_gps_fix_once = false;
}

#if SEND_MEKFW_FILTER || PERIODIC_TELEMETRY
static void send_mekfw_filter(struct transport_tx *trans, struct link_device *dev)
{
	struct FloatEulers eulers;
	float_eulers_of_quat(&eulers, &mekf_wind_float.state.quat);
	pprz_msg_send_MEKF_WIND(trans, dev,
			AC_ID,
			&mekf_wind_float.state.quat.qi,
			&mekf_wind_float.meas.baro_alt,
			&mekf_wind_float.meas.pos_gps.z);//?toute les mesures
}
#endif

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
	struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
	llh_nav0.lat = NAV_LAT0;
	llh_nav0.lon = NAV_LON0;
	/* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
	llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
	struct EcefCoor_i ecef_nav0;
	ecef_of_lla_i(&ecef_nav0, &llh_nav0);
	struct LtpDef_i ltp_def;
	ltp_def_from_ecef_i(&ltp_def, &ecef_nav0);
	ltp_def.hmsl = NAV_ALT0;
	stateSetLocalOrigin_i(&ltp_def);
#endif

	B.x = INS_H_X;
	B.y = INS_H_Y;
	B.z = INS_H_Z;

	// init state and measurements
	init_mekfw_state();



#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MEKF_FILTER, send_mekfw_filter);
#endif
}


void ins_reset_local_origin(void)
{
#if MEKFW_USE_UTM
	struct UtmCoor_f utm = utm_float_from_gps(&gps, 0);
	// reset state UTM ref
	stateSetLocalUtmOrigin_f(&utm);
#else
	struct LtpDef_i ltp_def;
	ltp_def_from_ecef_i(&ltp_def, &gps.ecef_pos);
	ltp_def.hmsl = gps.hmsl;
	stateSetLocalOrigin_i(&ltp_def);
#endif
}

void ins_reset_altitude_ref(void)
{
#if MEKFW_USE_UTM
	struct UtmCoor_f utm = state.utm_origin_f;
	utm.alt = gps.hmsl / 1000.0f;
	stateSetLocalUtmOrigin_f(&utm);
#else
	struct LlaCoor_i lla = {
			.lat = state.ned_origin_i.lla.lat,
			.lon = state.ned_origin_i.lla.lon,
			.alt = gps.lla_pos.alt
	};
	struct LtpDef_i ltp_def;
	ltp_def_from_lla_i(&ltp_def, &lla);
	ltp_def.hmsl = gps.hmsl;
	stateSetLocalOrigin_i(&ltp_def);
#endif
}

/*void mekf_wind_floatariant_align(struct FloatRates *lp_gyro,
                               struct FloatVect3 *lp_accel,
                               struct FloatVect3 *lp_mag)
{
  /* Compute an initial orientation from accel and mag directly as quaternion */
//ahrs_float_get_quat_from_accel_mag(&mekf_wind_float.state.quat, lp_accel, lp_mag);

/* use average gyro as initial value for bias */
//mekf_wind_float.state.bias = *lp_gyro;

/* push initial values to state interface */
//stateSetNedToBodyQuat_f(&mekf_wind_float.state.quat);

// ins and ahrs are now running
mekf_wind_float.is_aligned = true;
}

void mekf_wind_float_propagate(struct FloatRates* gyro, struct FloatVect3* accel, float dt)
{
	struct FloatRates body_rates;
	// fill command vector in body frame
	struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&mekf_wind_float.body_to_imu);
	float_rmat_transp_ratemult(&mekf_wind_float.cmd.rates, body_to_imu_rmat, gyro);
	float_rmat_transp_vmult(&mekf_wind_float.cmd.accel, body_to_imu_rmat, accel);

#if SEND_MEKFW_FILTER
	RunOnceEvery(3, send_mekfw_filter(&(DefaultChannel).trans_tx, &(DefaultDevice).device));
#endif

	// concerve uniquement les mesures gyro et accéléro et t ?
#if LOG_MEKFW_FILTER
	if (pprzLogFile != -1) {
		sdLogWriteLog(pprzLogFile,
				"%.3f",
				get_sys_time_float());
		sdLogWriteLog(pprzLogFile,
				" gyro_accel ");
		sdLogWriteLog(pprzLogFile,
				"%.3f %.3f %.3f %.3f %.3f %.3f \n",
				mekf_wind_float.cmd.rates.p,
				mekf_wind_float.cmd.rates.q,
				mekf_wind_float.cmd.rates.r,
				mekf_wind_float.cmd.accel.x,
				mekf_wind_float.cmd.accel.y,
				mekf_wind_float.cmd.accel.z,
		);
	}
#endif
}

void mekf_wind_float_update_gps(struct GpsState *gps_s)
{
	if (gps_s->fix >= GPS_FIX_3D && mekf_wind_float.is_aligned) {
		ins_gps_fix_once = true;

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
#if LOG_MEKFW_FILTER
		if (pprzLogFile != -1) {
			// log file header
			sdLogWriteLog(pprzLogFile,
					"%.3f",
					get_sys_time_float()):
						;
			sdLogWriteLog(pprzLogFile,
					" gps ");
			log_started = true;
			sdLogWriteLog(pprzLogFile,
					"%.3f %.3f %.3f %.3f %.3f %.3f \n",
					mekf_wind_float.meas.pos_gps.x,
					mekf_wind_float.meas.pos_gps.y,
					mekf_wind_float.meas.pos_gps.z,
					mekf_wind_float.meas.speed_gps.x,
					mekf_wind_float.meas.speed_gps.y,
					mekf_wind_float.meas.speed_gps.z,
			);
		}
#endif
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

}


void mekf_wind_float_update_baro(float pressure)
{
	static float ins_qfe = 101325.0f;
	static float alpha = 10.0f;
	static int32_t i = 1;
	static float baro_moy = 0.0f;
	static float baro_prev = 0.0f;

	if (!ins_baro_initialized) {
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
			ins_baro_initialized = true;
		}
		if (i == 250) {
			ins_qfe = pressure;
			ins_baro_initialized = true;
		}
		i++;
	} else { /* normal update with baro measurement */
		mekf_wind_float.meas.baro_alt = -pprz_isa_height_of_pressure(pressure, ins_qfe); // Z down
	}
	//nouveau code pour envoyer sur log
#if LOG_MEKFW_FILTER
	if (pprzLogFile != -1) {
			// log file header
			sdLogWriteLog(pprzLogFile,
					"%.3f",
					get_sys_time_float());
			sdLogWriteLog(pprzLogFile,
					" baro ");
			log_started = true;
			sdLogWriteLog(pprzLogFile,
					"%.3f \n",
					mekf_wind_float.meas.baro_alt,
			);
	}
#endif
	//fin nouveau    
}

// assume mag is dead when values are not moving anymore
#define MAG_FROZEN_COUNT 30

void mekf_wind_float_update_mag(struct FloatVect3* mag)
{
	static uint32_t mag_frozen_count = MAG_FROZEN_COUNT;
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
		struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&mekf_wind_float.body_to_imu);
		// new values in body frame
		float_rmat_transp_vmult(&mekf_wind_float.meas.mag, body_to_imu_rmat, mag);
		// reset counter
		mag_frozen_count = MAG_FROZEN_COUNT;
	}
	last_mx = mag->x;
	//nouveau code pour envoyer sur log
#if LOG_MEKFW_FILTER
	if (pprzLogFile != -1) {
			sdLogWriteLog(pprzLogFile,
					"%.3f"
					get_sys_time_float());
			sdLogWriteLog(pprzLogFile,
					" magneto ");
			log_started = true;
		} else {
			sdLogWriteLog(pprzLogFile,
					"%.3f %.3f %.3f\n",
					mekf_wind_float.meas.mag.x,
					mekf_wind_float.meas.mag.y,
					mekf_wind_float.meas.mag.z,
			);
	}
#endif
	//fin nouveau  	
}


static inline void mekf_wind_model(float *o, const float *x, const int n, const float *u,
		const int m __attribute__((unused)))
{

#pragma GCC diagnostic push // require GCC 4.6
#pragma GCC diagnostic ignored "-Wcast-qual"
	struct mekf_wind_command *c = (struct mekf_wind_command *)u;
#pragma GCC diagnostic pop // require GCC 4.6
	struct FloatRates rates_unbiased;
	struct FloatVect3 tmp_vect;


	// set output
	memcpy(o, &s_dot, n * sizeof(float));
}



