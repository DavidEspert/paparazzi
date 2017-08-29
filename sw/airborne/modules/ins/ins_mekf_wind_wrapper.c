/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/ins/ins_mekf_wind_wrapper.c
 *
 * Paparazzi specific wrapper to run MEKF-Wind INS filter.
 */

#include "modules/ins/ins_mekf_wind_wrapper.h"
#include "modules/ins/ins_mekf_wind.h"
#include "subsystems/abi.h"
#include "math/pprz_isa.h"
#include "state.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

#define MEKF_WIND_USE_UTM TRUE
#if MEKF_WIND_USE_UTM
#include "firmwares/fixedwing/nav.h"
#endif

#ifndef INS_MEKF_WIND_FILTER_ID
#define INS_MEKF_WIND_FILTER_ID 3
#endif

/** last accel measurement */
static struct FloatVect3 ins_mekf_wind_accel;
static uint32_t last_imu_stamp = 0;

static void set_body_state_from_quat(void);

#undef PERIODIC_TELEMETRY

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"

static void send_euler(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatEulers ltp_to_imu_euler;
  float_eulers_of_quat(&ltp_to_imu_euler, &ins_mekf_wind.ltp_to_imu_quat);
  pprz_msg_send_AHRS_EULER(trans, dev, AC_ID,
                           &ltp_to_imu_euler.phi,
                           &ltp_to_imu_euler.theta,
                           &ltp_to_imu_euler.psi,
                           &ins_mekf_wind.id);
}

static void send_bias(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Rates gyro_bias;
  RATES_BFP_OF_REAL(gyro_bias, ins_mekf_wind.gyro_bias);
  pprz_msg_send_AHRS_GYRO_BIAS_INT(trans, dev, AC_ID,
                                   &gyro_bias.p, &gyro_bias.q, &gyro_bias.r, &ins_mekf_wind.id);
}

static void send_geo_mag(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GEO_MAG(trans, dev, AC_ID,
                        &ins_mekf_wind.mag_h.x, &ins_mekf_wind.mag_h.y, &ins_mekf_wind.mag_h.z, &ins_mekf_wind.id);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mde = 3;
  uint16_t val = 0;
  if (!ins_mekf_wind.is_aligned) { mde = 2; }
  uint32_t t_diff = get_sys_time_usec() - last_imu_stamp;
  /* set lost if no new gyro measurements for 50ms */
  if (t_diff > 50000) { mde = 5; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ins_mekf_wind.id, &mde, &val);
}
#endif

/*
 * ABI bindings
 */

/** airspeed (Pitot tube) */
#ifndef INS_MEKF_IND_AIRSPEED_ID
#define INS_MEKF_IND_AIRSPEED_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_MEKF_IND_AIRSPEED_ID)


/** baro */
#ifndef INS_MEKF_WIND_BARO_ID
#if USE_BARO_BOARD
#define INS_MEKF_WIND_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_MEKF_WIND_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_MEKF_WIND_BARO_ID)

/** IMU (gyro, accel) */
#ifndef INS_MEKF_WIND_IMU_ID
#define INS_MEKF_WIND_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_MEKF_WIND_IMU_ID)

/** magnetometer */
#ifndef INS_MEKF_WIND_MAG_ID
#define INS_MEKF_WIND_MAG_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_MEKF_WIND_MAG_ID)

/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef INS_MEKF_WIND_GPS_ID
#define INS_MEKF_WIND_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_MEKF_WIND_GPS_ID)

static abi_event pressure_diff_ev;
static abi_event baro_ev;
static abi_event mag_ev;
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event aligner_ev;
static abi_event body_to_imu_ev;
static abi_event geo_mag_ev;
static abi_event gps_ev;



static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure)
{
  ins_mekf_wind_update_baro(pressure);
}

static void pressure_diff_cb(uint8_t __attribute__((unused)) sender_id, float pdyn)
{
  float airspeed = eas_from_dynamic_pressure(pdyn);
  ins_mekf_wind_update_airspeed(airspeed);
}

/**
 * Call ins_mekf_wind_propagate on new gyro measurements.
 * Since acceleration measurement is also needed for propagation,
 * use the last stored accel from #ins_mekfw_accel.
 */
static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp, struct Int32Rates *gyro)
{
  struct FloatRates gyro_f;
  RATES_FLOAT_OF_BFP(gyro_f, *gyro);

#if USE_AUTO_INS_FREQ || !defined(INS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for INS MEKF_WIND propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ins_mekf_wind_propagate(&gyro_f, &ins_mekf_wind_accel, dt);
    set_body_state_from_quat();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed INS_PROPAGATE_FREQUENCY for INS MEKF_WIND propagation.")
  PRINT_CONFIG_VAR(INS_PROPAGATE_FREQUENCY)
  const float dt = 1. / (INS_PROPAGATE_FREQUENCY);
  ins_mekf_wind_propagate(&gyro_f, &ins_mekf_wind_accel, dt);
#endif
  last_imu_stamp = last_stamp;
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  ACCELS_FLOAT_OF_BFP(ins_mekf_wind_accel, *accel);
}

static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct Int32Vect3 *mag)
{
  if (ins_mekf_wind.is_aligned) {
    struct FloatVect3 mag_f;
    MAGS_FLOAT_OF_BFP(mag_f, *mag);
    ins_mekf_wind_update_mag(&mag_f);
    set_body_state_from_quat();
  }
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ins_mekf_wind.is_aligned) {
    /* convert to float */
    struct FloatRates gyro_f;
    RATES_FLOAT_OF_BFP(gyro_f, *lp_gyro);
    struct FloatVect3 accel_f;
    ACCELS_FLOAT_OF_BFP(accel_f, *lp_accel);
    struct FloatVect3 mag_f;
    MAGS_FLOAT_OF_BFP(mag_f, *lp_mag);
    /* set initial body orientation in state interface if alignment was successful */
    if (ins_mekf_wind_align(&gyro_f, &accel_f, &mag_f)) {
      set_body_state_from_quat();
    }
  }
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  ins_mekf_wind_set_body_to_imu_quat(q_b2i_f);
}

static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{
  ins_mekf_wind.mag_h = *h;
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
	if (gps_s->fix >= GPS_FIX_3D) {
    ins_mekf_wind.gps_fix_once = true;

#if MEKF_WIND_USE_UTM
		if (state.utm_initialized_f) {
			struct UtmCoor_f utm = utm_float_from_gps(gps_s, nav_utm_zone0);
      struct FloatVect3 pos, speed;
			// position (local ned)
			pos.x = utm.north - state.utm_origin_f.north;
			pos.y = utm.east - state.utm_origin_f.east;
			pos.z = state.utm_origin_f.alt - utm.alt;
			// speed
			speed.x = gps_s->ned_vel.x / 100.0f;
			speed.y = gps_s->ned_vel.y / 100.0f;
			speed.z = gps_s->ned_vel.z / 100.0f;
      ins_mekf_wind_update_pos_speed(&pos, &speed);
		}

#else
		if (state.ned_initialized_f) {
      struct FloatVect3 pos, speed;
			struct NedCoor_i gps_pos_cm_ned, ned_pos;
			ned_of_ecef_point_i(&gps_pos_cm_ned, &state.ned_origin_i, &gps_s->ecef_pos);
			INT32_VECT3_SCALE_2(ned_pos, gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
			NED_FLOAT_OF_BFP(pos, ned_pos);
			struct EcefCoor_f ecef_vel;
			ECEF_FLOAT_OF_BFP(ecef_vel, gps_s->ecef_vel);
			ned_of_ecef_vect_f(&speed, &state.ned_origin_f, &ecef_vel);
      ins_mekf_wind_update_pos_speed(&pos, &speed);
		}
#endif
	}
}

void ins_mekf_wind_aoa_periodic(void)
{
  float aoa = stateGetAngleOfAttack_f();
  float aos = stateGetSideslip_f();
  ins_mekf_wind_update_incidence(aoa, aos);
#if USE_NPS || USE_AIRSPEED_PERIODIC
  ins_mekf_wind_update_airspeed(stateGetAirspeed_f());
#endif
}

/**
 * Compute body orientation and rates from imu orientation and rates
 */
static void set_body_state_from_quat(void)
{
//  struct FloatQuat *body_to_imu_quat = orientationGetQuat_f(&ins_mekf_wind.body_to_imu);
//  struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ins_mekf_wind.body_to_imu);
//
//  /* Compute LTP to BODY quaternion */
//  struct FloatQuat ltp_to_body_quat;
//  float_quat_comp_inv(&ltp_to_body_quat, &ins_mekf_wind.ltp_to_imu_quat, body_to_imu_quat);
//  /* Set in state interface */
//  stateSetNedToBodyQuat_f(&ltp_to_body_quat);
//
//  /* compute body rates */
//  struct FloatRates body_rate;
//  float_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &ins_mekf_wind.imu_rate);
//  /* Set state */
//  stateSetBodyRates_f(&body_rate);
}

/**
 * Init function
 */
void ins_mekf_wind_wrapper_init(void)
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

  // init filter
  ins_mekf_wind_init();

  // Bind to ABI messages
  AbiBindMsgBARO_ABS(INS_MEKF_WIND_BARO_ID, &baro_ev, baro_cb);
  AbiBindMsgBARO_DIFF(INS_MEKF_WIND_AIRSPEED_ID, &pressure_diff_ev, pressure_diff_cb);
  AbiBindMsgIMU_MAG_INT32(INS_MEKF_WIND_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_GYRO_INT32(INS_MEKF_WIND_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(INS_MEKF_WIND_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_LOWPASSED(INS_MEKF_WIND_IMU_ID, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(INS_MEKF_WIND_IMU_ID, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &geo_mag_ev, geo_mag_cb);
  AbiBindMsgGPS(INS_MEKF_WIND_GPS_ID, &gps_ev, gps_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_EULER, send_euler);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_GYRO_BIAS_INT, send_bias);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GEO_MAG, send_geo_mag);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
#endif
}

