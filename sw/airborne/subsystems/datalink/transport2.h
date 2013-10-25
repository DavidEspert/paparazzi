/*
 * Copyright (C) 2003-2010 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file transport2.h
 *  \brief Interface for downlink transport implementation
 * 
 * Each transport layer 'X' must contain 4 functions:
 * - uint8_t X_header_len(void):
 * 	Returns header length.
 * - void X_header(uint8_t *buff, uint8_t msg_data_length, uint8_t msg_id):
 * 	Fills header in buffer.
 * - uint8_t X_tail_len(void):
 * 	Returns tail length.
 * - void X_tail(uint8_t *buff, uint8_t msg_data_length):
 * 	Fills tail in buffer. It must internally take into account the offset in buffer
 * 	(offset = header_len + msg_data_length)
 */

#ifndef DOWNLINK_TRANSPORT_H
#define DOWNLINK_TRANSPORT_H

//#define _USE_INLINE_TRANSPORT_
#ifdef _USE_INLINE_TRANSPORT_

//transport_X_inline.h contain the same functions than transport_X.c but defined as 'static inline'.
// So, the user can directly call the functions without requiring any 'Transport2' struct.
#include "transport_pprz_inline.h"
#include "transport_xbee_inline.h"
#include "transport_ivy_inline.h"
#include "transport_default_inline.h"

#else

#include <inttypes.h>
#include "std.h"
//#include <stdint.h>

struct DownlinkTransport
{
	uint8_t (*header_len)(void);
	void (*header)(uint8_t *buff, uint8_t msg_data_length, uint8_t msg_id);
	uint8_t (*tail_len)(void);
	void (*tail)(uint8_t *buff, uint8_t msg_data_length);
	void (*parse)(void);
	// message received flag
	volatile bool_t msg_received;
	// overrun and error flags
	uint8_t ovrn, error;
};

// declaration of developed transport layer structs
extern struct DownlinkTransport PprzTransport;
extern struct DownlinkTransport XBeeTransport;
extern struct DownlinkTransport IvyTransport;

#endif // _USE_INLINE_TRANSPORT_

#endif /* DOWNLINK_TRANSPORT_H */
