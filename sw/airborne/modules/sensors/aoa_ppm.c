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
 * @file "modules/sensors/aoa_ppm.h"
 * @author Jean-François Erdelyi
 * @brief Angle of Attack sensor on PPM
 * 
 * SENSOR, exemple : US DIGITAL MA3-P12-125-B
 */

#include "modules/sensors/aoa_ppm.h"
#include "mcu_periph/pwm_input.h"
#include "subsystems/datalink/downlink.h"

#ifdef LOG_AOS_MS
	#include "sdLog.h"
	#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
	bool_t log_started;
#endif

#ifndef PWM_PERIOD
	#define PWM_PERIOD 4098
#endif

#ifndef PWM_CHANNEL_AOA
	#error "PWM_CHANNEL_AOA needs to be defined to use aoa_ppm module"
#endif

struct Aoa_Ppm aoa_ppm;

void aoa_ppm_init(void) {
	aoa_ppm.angle = 0.0;
	aoa_ppm.raw = 0.0;
	#if LOG_AOS_MS
		log_started = FALSE;
	#endif
}

// Documentation of sensor US DIGITAL MA3-P12-125-B
// @see http://www.usdigital.com/products/encoders/absolute/rotary/shaft/ma3
void aoa_ppm_update(void) {
	float duty_raw = pwm_input_duty_tics[PWM_CHANNEL_AOA];
	float duty = (duty_raw / (PWM_PERIOD - 2)); // Converted to %
	float t_on = duty * PWM_PERIOD;
	float t_off = PWM_PERIOD - t_on;
	float raw_data = ((t_on * PWM_PERIOD) / (t_on + t_off)) - 1;

	if(raw_data <= PWM_PERIOD - 4) {
		aoa_ppm.raw = raw_data;
	} else if(raw_data == PWM_PERIOD - 2) {
		aoa_ppm.raw = PWM_PERIOD - 3;
	}
	aoa_ppm.angle = ((aoa_ppm.raw / PWM_PERIOD) * 360) - 180;
	DOWNLINK_SEND_AOA_PPM(DefaultChannel, DefaultDevice, &aoa_ppm.angle, &aoa_ppm.raw);

	#if LOG_AOS_MS
		if(pprzLogFile != -1) {
			if (!log_started) {
				sdLogWriteLog(pprzLogFile, "AOA : ANGLE, RAW\n");
				log_started = TRUE;
			} else {
				sdLogWriteLog(pprzLogFile, "AOA : %f, %d\n", aoa_ppm.angle, aoa_ppm.raw);
			}
		}
	#endif
}
