/*
 * Copyright (C) Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * @file "modules/ctrl/face_tracking.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control a rotorcraft heading to track a face detected by a camera
 */

#include "modules/ctrl/face_tracking.h"

#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/abi.h"
#include "generated/airframe.h"
#include "generated/modules.h"

// ABI message binding ID
#ifndef FACE_TRACKING_ID
#define FACE_TRACKING_ID ABI_BROADCAST
#endif

// Timout in seconds before entering search mode
#ifndef FACE_TRACKING_TIMEOUT
#define FACE_TRACKING_TIMEOUT 3.0f
#endif

// Deadband (constant heading setpoint)
#ifndef FACE_TRACKING_DEADBAND
#define FACE_TRACKING_DEADBAND 100
#endif

// Turn rate in tracking mode (rad/s)
#ifndef FACE_TRACKING_RATE
#define FACE_TRACKING_RATE RadOfDeg(10)
#endif

// Turn rate in search mode (rad/s)
#ifndef FACE_TRACKING_SEARCH_RATE
#define FACE_TRACKING_SEARCH_RATE RadOfDeg(20)
#endif

// Camera horizontal FOV
#ifndef FACE_TRACKING_HFOV
#define FACE_TRACKING_HFOV RadOfDeg(45)
#endif

// Maximum value (JeVois normalized messages between -1000 and 1000)
#ifndef FACE_TRACKING_MAX
#define FACE_TRACKING_MAX 1000
#endif

// Select rate or heading tracking mode
#ifndef FACE_TRACKING_USE_RATE
#define FACE_TRACKING_USE_RATE FALSE
#endif

// Send debug message
#ifndef FACE_TRACKING_DEBUG
#define FACE_TRACKING_DEBUG TRUE
#endif

#if FACE_TRACKING_DEBUG
#include "subsystems/datalink/downlink.h"
#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"
#endif

int16_t face_tracking_deadband;
float face_tracking_rate;
float face_tracking_search_rate;

int16_t face_lat;
int16_t face_vert;
float timeout;

abi_event face_ev;

static const float nav_dt = 1. / NAV_FREQ;

static void get_face(uint8_t sender_id __attribute__((unused)),
    uint8_t type __attribute__((unused)),
    char *id __attribute__((unused)),
    uint8_t nb __attribute__((unused)),
    int16_t *coord,
    uint16_t *dim __attribute__((unused)),
    struct FloatQuat quat __attribute__((unused)),
    char *extra __attribute__((unused))
    )
{
  if (type == JEVOIS_MSG_T2) {
    face_lat = coord[0];
    face_vert = coord[1];
    timeout = 0.f;
  }
}

void face_tracking_init(void)
{
  face_tracking_deadband = FACE_TRACKING_DEADBAND;
  face_tracking_search_rate = FACE_TRACKING_SEARCH_RATE;
  face_tracking_rate = FACE_TRACKING_RATE;

  face_lat = 0;
  face_vert = 0;
  timeout = FACE_TRACKING_TIMEOUT; // start in search mode

  // Bind to camera message
  AbiBindMsgJEVOIS_MSG(FACE_TRACKING_ID, &face_ev, get_face);
}

void face_tracking_run(void)
{
#if FACE_TRACKING_USE_RATE
  float rate = face_tracking_search_rate; // search mode
  if (timeout < FACE_TRACKING_TIMEOUT) {
    timeout += nav_dt;
    if (face_lat > face_tracking_deadband) {
      rate = face_tracking_rate;
    } else if (face_lat < -face_tracking_deadband) {
      rate = -face_tracking_rate;
    } else {
      rate = 0.f;
    }
#if FACE_TRACKING_DEBUG
    float msg[] = {
      rate,
      (float)face_lat,
      (float)face_vert,
      (float)timeout
    };
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, msg);
#endif
  }
  nav_heading += ANGLE_BFP_OF_REAL(rate * nav_dt);

#else // use calculated heading

  if (timeout < FACE_TRACKING_TIMEOUT) {
    timeout += nav_dt;
    // compute expected heading
    float cam_heading = (FACE_TRACKING_HFOV / (2.f * FACE_TRACKING_MAX)) * (float)face_lat;
    float target_heading = stateGetNedToBodyEulers_f()->psi + cam_heading;
    float diff = target_heading - ANGLE_FLOAT_OF_BFP(nav_heading);
    FLOAT_ANGLE_NORMALIZE(diff);
    BoundAbs(diff, face_tracking_rate * nav_dt)
    nav_heading += ANGLE_BFP_OF_REAL(diff);
#if FACE_TRACKING_DEBUG
    float msg[] = {
      target_heading,
      (float)face_lat,
      (float)face_vert,
      (float)timeout
    };
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, msg);
#endif
  } else {
    nav_heading += ANGLE_BFP_OF_REAL(face_tracking_search_rate * nav_dt);
  }
#endif
}

