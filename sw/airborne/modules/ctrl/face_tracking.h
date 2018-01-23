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
 * @file "modules/ctrl/face_tracking.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control a rotorcraft heading to track a face detected by a camera
 */

#ifndef FACE_TRACKING_H
#define FACE_TRACKING_H

#include "std.h"

extern int16_t face_tracking_deadband;
extern float face_tracking_search_rate;
extern float face_tracking_rate;

extern void face_tracking_init(void);
extern void face_tracking_run(void);

#endif

