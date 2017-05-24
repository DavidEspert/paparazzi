/*
 * Copyright (C) 2015 Felix Ruess <felix.ruess@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/mekf_wind/mekf_wind_float_wrapper.h
 *
 * Paparazzi specific wrapper to run MEKF-Wind filter.
 */

#ifndef MEKF_WIND_FLOAT_WRAPPER_H
#define MEKF_WIND_FLOAT_WRAPPER_H

#include "modules/mekf_wind/mekf_wind_float.h"

extern void mekf_wind_float_wrapper_init(void);
extern void mekf_wind_aoa_periodic(void);

#endif /* MEKF_WIND_FLOAT_WRAPPER_H */

