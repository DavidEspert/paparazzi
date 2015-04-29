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

#ifndef AOA_PPM_H
#define AOA_PPM_H

#include "std.h"

struct Aoa_Ppm {
	float angle;
	float raw;
};

extern struct Aoa_Ppm aoa_ppm;

extern void aoa_ppm_init(void);
extern void aoa_ppm_update(void);

#endif /*AOA_PPM_H*/

