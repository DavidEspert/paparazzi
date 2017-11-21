/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/computer_vision/jevois.h"
 * @author Gautier Hattenberger
 * Decoder for standardized messages from the JEVOIS smart camera (http://jevois.org)
 */

#ifndef JEVOIS_H
#define JEVOIS_H

#define JEVOIS_MSG_T1 10
#define JEVOIS_MSG_N1 11
#define JEVOIS_MSG_D1 12
#define JEVOIS_MSG_T2 20
#define JEVOIS_MSG_N2 21
#define JEVOIS_MSG_D2 22
#define JEVOIS_MSG_F2 23
#define JEVOIS_MSG_T3 30
#define JEVOIS_MSG_N3 31
#define JEVOIS_MSG_D3 32
#define JEVOIS_MSG_F3 33

/** Normalized data from JEVOIS are between -1000 and 1000 */
#define JEVOIS_NORM 1000

extern void jevois_init(void);
extern void jevois_event(void);

#endif

