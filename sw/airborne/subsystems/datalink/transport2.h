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
 *  \brief Interface for transport implementation
 * 
 * This file provides a generic interface for transport layer control.
 * General transport layer is defined with a void 'data' struct (containing
 * specific and common transport data) and API functions.
 * 
 * IMPORTANT NOTE: Since system data adquisition is not synchronized, Rx parsed data
 * could be a truncated message. In order to keep Rx integrity, transport layer has to ALWAYS
 * be linked to the same device; in that way, every rx data array will be concatenated
 * with the previous one and the truncation problem will disapear.
 * i.e.: if you want 2 Paparazzi transports over 2 devices you need 2 'transport2'
 *       elements of Paparazzi type.
 * |---> Uplink manager (datalink.h) ensures that same transport is not associated to
 *       different devices.
 *
 * NOTE 2: If different services expect data from the same device and with the same transport,
 * it is not necessary to create different 'transport2' elements. Create just one and add their
 * 'service callback' to it.
 */

#ifndef _TRANSPORT2_H
#define _TRANSPORT2_H

#include "device.h"

#define _TRANSPORT_TRACES_

#ifdef _TRANSPORT_TRACES_
#include <stdio.h>
#define TRANSPORT_TRACE(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#define TRANSPORT_PRINT_PAYLOAD(_common) { \
  uint16_t payload_idx; \
  if (_common.payload_len > 0) { \
    TRANSPORT_TRACE("\ttransport2: Rx message:    { "); \
    for(payload_idx = 0; payload_idx < (_common.payload_len-1); payload_idx++){ \
      TRANSPORT_TRACE("%u, ", _common.payload[payload_idx]); \
    } \
    TRANSPORT_TRACE("%u }\n", _common.payload[payload_idx]); \
  } \
}
#else
#define TRANSPORT_TRACE(...)
#define TRANSPORT_PRINT_PAYLOAD(_common)
#endif


// TX API ---------------------------------------------------------------------
struct transport_tx_api {
  uint8_t         header_len;
  void            (*header)(uint8_t *buff, uint16_t msg_data_length);
  uint8_t         tail_len;
  void            (*tail)(uint8_t *buff, uint16_t msg_data_length);
};

struct transport_tx {
  struct transport_tx_api api;
};

#ifdef TRANSPORT_TX_1
extern struct transport_tx transport_tx_1;
#endif
#ifdef TRANSPORT_TX_2
extern struct transport_tx transport_tx_2;
#endif

extern struct transport_tx PprzTransport;
extern struct transport_tx XBeeTransport;


// RX API (and RX_DATA) -------------------------------------------------------
#define TP_NAME_SIZE            16
#ifndef TRANSPORT_PAYLOAD_LEN
#define TRANSPORT_PAYLOAD_LEN   256
#endif
#ifndef TRANSPORT_NUM_CALLBACKS
#define TRANSPORT_NUM_CALLBACKS 1
#endif

struct transport_rx_data_common {
  // transport name
  char name[TP_NAME_SIZE];
  // payload buffer
  uint8_t payload[TRANSPORT_PAYLOAD_LEN];
  // payload length
  volatile uint16_t payload_len;
  // message received flag
  volatile bool_t msg_received;
  // overrun and error flags
  uint8_t ovrn, error;
  //callback functions 
  void (*callback[TRANSPORT_NUM_CALLBACKS])(const uint8_t*payload, const uint16_t payload_len);
  // associated Rx device
  struct device* rx_dev;
};
#define INITIALIZED_TP_RX_DATA_COMMON(_name) { \
  .name = _name, \
  .payload_len = 0, \
  .msg_received = FALSE, \
  .ovrn = 0, \
  .error = 0, \
  .callback[0 ... (TRANSPORT_NUM_CALLBACKS-1)] = NULL, \
  .rx_dev = NULL \
}

static inline void transport_rx_data_common_init(struct transport_rx_data_common* common) {
  common->payload_len = 0;
  common->msg_received = FALSE;
  common->ovrn = 0;
  common->error = 0;
  for (uint8_t i = 0; i < TRANSPORT_NUM_CALLBACKS; i++)
    common->callback[i] = NULL;
  common->rx_dev = NULL;
}

struct device;  //even if "device.h" is included, forward definition is required due to circular dependencies.
struct transport_rx_api {
  void            (*init)(void* data);
  bool_t          (*register_device)(void* data, struct device* rx_dev);  //register always through datalink.h!
  struct device*  (*rx_device)(void* data);
  char*           (*name)(void* data);
  void            (*parse)(void* data, uint8_t byte);
  bool_t          (*message_received)(void* data);
  bool_t          (*register_callback)(void* data, void (*callback)(const uint8_t*, const uint16_t) );
  void            (*callback)(void* data);
};

struct transport_rx{
  void * data; //this points to the transport data struct
  struct transport_rx_api api;
};

#ifdef TRANSPORT_RX_1
extern struct transport_rx transport_rx_1;
#endif
#ifdef TRANSPORT_RX_2
extern struct transport_rx transport_rx_2;
#endif


#endif /* _TRANSPORT2_H */
