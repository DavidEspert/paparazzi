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
 * - void X_header(uint8_t *buff, uint16_t msg_data_length):
 * 	Fills header in buffer.
 * - uint8_t X_tail_len(void):
 * 	Returns tail length.
 * - void X_tail(uint8_t *buff, uint16_t msg_data_length):
 * 	Fills tail in buffer. It must internally take into account the offset in buffer
 * 	(offset = header_len + msg_data_length)
 */

#ifndef _TRANSPORT2_H
#define _TRANSPORT2_H

// #include <inttypes.h>
// #include "std.h"
// #include <stdint.h>
#include "device.h"

#define _TRANSPORT_TRACES_

#ifdef _TRANSPORT_TRACES_
#include <stdio.h>
#define TRANSPORT_TRACE(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#else
#define TRANSPORT_TRACE(...)
#endif


#define TP_NAME_SIZE            16
#ifndef TRANSPORT_PAYLOAD_LEN
#define TRANSPORT_PAYLOAD_LEN   256
#endif
#ifndef TRANSPORT_NUM_CALLBACKS
#define TRANSPORT_NUM_CALLBACKS 1
#endif

/** Generic Rx data struct */
struct transport_data {
  // transport name
  char name[TP_NAME_SIZE];
  // payload buffer
  uint8_t payload[TRANSPORT_PAYLOAD_LEN];
  // payload length
  volatile uint16_t payload_len;
  // message received flag
//   volatile bool_t msg_received;
  // overrun and error flags
//   uint8_t ovrn, error;
  uint8_t error;
  //callback functions 
  void (*callback[TRANSPORT_NUM_CALLBACKS])(const uint8_t*payload, const uint16_t payload_len);
  // associated Rx device
  struct device* rx_dev;
};

/** Generic Transport API */
struct transport_api
{
  void            (*init)(void* tp_struct, struct device* rx_dev);
  struct device*  (*rx_device)(void* tp_struct);
  char*           (*name)(void* tp_struct);
  // TX functions
  uint8_t         header_len;
  void            (*header)(uint8_t *buff, uint16_t msg_data_length);
  uint8_t         tail_len;
  void            (*tail)(uint8_t *buff, uint16_t msg_data_length);
  // RX functions
  bool_t          (*register_callback)(void* tp_struct, void (*callback)(const uint8_t*, const uint16_t) );
  void            (*parse)(void* tp_struct, uint8_t *c, uint16_t length);
};

/** Generic Transport interface */
struct transport2
{
  void * tp_data; //this points to the transport data struct
  struct transport_api api;
};

#endif /* _TRANSPORT2_H */
