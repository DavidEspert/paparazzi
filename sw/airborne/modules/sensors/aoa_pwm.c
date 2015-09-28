/*
* Copyright (C) Jean-François Erdelyi
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
* @file "modules/sensors/aoa_pwm.h"
* @author Jean-François Erdelyi
* @brief Angle of Attack sensor on PWM
* 
* SENSOR, exemple : US DIGITAL MA3-P12-125-B
*/

#include "modules/sensors/aoa_pwm.h"
#include "mcu_periph/pwm_input.h"
#include "subsystems/datalink/downlink.h"

#ifdef AOA_LOG_MS
#include "sdLog.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
bool_t log_started;
#endif

#ifndef AOA_PWM_PERIOD
#error "AOA_PWM_PERIOD needs to be defined to use aoa_pwm module"
#endif

#ifndef AOA_PWM_CHANNEL
#error "AOA_PWM_CHANNEL needs to be defined to use aoa_pwm module"
#endif

#ifndef AOA_OFFSET
#define AOA_OFFSET 0
#endif

#ifndef AOA_SIGN
#define AOA_SIGN 1 
#endif

struct Aoa_Pwm aoa_pwm;

void aoa_pwm_init(void) {
  aoa_pwm.angle = 0.0;
  aoa_pwm.raw = 0.0;
#if AOA_LOG_MS
  log_started = FALSE;
#endif
}

// Documentation of sensor US DIGITAL MA3-P12-125-B
// @see http://www.usdigital.com/products/encoders/absolute/rotary/shaft/ma3
// For the calibration don't forget : 0 degree is not 0 in raw values but 4096/2=2048
// Because the measure is not 0 to 180 degree but -180 to 180 degree
void aoa_pwm_update(void) {
  int16_t duty_raw = pwm_input_duty_tics[AOA_PWM_CHANNEL] - AOA_OFFSET;
  
  if(duty_raw < 0) {
    duty_raw = AOA_PWM_PERIOD - abs(duty_raw);
  }
  
  if(duty_raw <= AOA_PWM_PERIOD - 4) {
    aoa_pwm.raw = duty_raw;
  } else if(duty_raw == AOA_PWM_PERIOD - 2) {
    aoa_pwm.raw = AOA_PWM_PERIOD - 3;
  }

  aoa_pwm.angle = AOA_SIGN * ((((float)duty_raw / AOA_PWM_PERIOD) * 360.f) - 180.f);
  DOWNLINK_SEND_AOA_PWM(DefaultChannel, DefaultDevice, &aoa_pwm.angle, &aoa_pwm.raw);

#if AOA_LOG_MS
    if(pprzLogFile != -1) {
      if (!log_started) {
        sdLogWriteLog(pprzLogFile, "AOA : ANGLE(deg), RAW(int16)\n");
        log_started = TRUE;
      } else {
        sdLogWriteLog(pprzLogFile, "AOA : %.3f, %d\n", aoa_pwm.angle, aoa_pwm.raw);
      }
    }
#endif
}
