/*
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
/** \file subsystems/datalink/datalink.h
 *  \brief Handling of messages coming from ground and other A/Cs.
 * 
 * This file manages Rx datalinks for onboard services. A datalink is defined with
 * a device, a transport layer and a parse function that selects the service messages.
 * 
 * Datalink manager can hold different transports for each device. Parse functions
 * will be stored by transport:
 * -Datalink --> Device1 --> transport1-1 --> parse1-1-1
 *           |           |                |-> parse2-1-1
 *           |           |
 *           |           |-> transport2-1 --> parse1-2-1
 *           |                            |-> parse2-2-1
 *           |
 *           |-> Device2 ...
 * 
 * CAUTION: transport layer has to be linked to a device before datalink registration.
 *  (Link is done by transport's init function)
 */

#ifndef DATALINK_H
#define DATALINK_H

#ifdef DATALINK_C
#define EXTERN
#else
#define EXTERN extern
#endif

#define _DATALINK_TRACES_

#ifdef _DATALINK_TRACES_
#include <stdio.h>
#define DATALINK_TRACE(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#else
#define DATALINK_TRACE(...)
#endif

// #ifdef __IEEE_BIG_ENDIAN /* From machine/ieeefp.h */
// #define Swap32IfBigEndian(_u) { _u = (_u << 32) | (_u >> 32); }
// #else
// #define Swap32IfBigEndian(_) {}
// #endif

#include "std.h"
#include "dl_protocol.h"

EXTERN bool_t dl_msg_available;
/** Flag provided to control calls to ::dl_parse_msg. NOT used in this module*/

EXTERN uint16_t datalink_time;

#define MSG_SIZE 128
EXTERN uint8_t dl_buffer[MSG_SIZE]  __attribute__ ((aligned));

EXTERN void dl_parse_msg(void);
/** Should be called when chars are available in dl_buffer */


#define NUM_DATALINK_DEV                2
#define DATALINK_DEV_NOT_REGISTERED     NUM_DATALINK_DEV
#define NUM_DATALINK_TP                 2
#define DATALINK_TP_NOT_REGISTERED      NUM_DATALINK_TP

struct Datalink_device{
  struct device* dev;
  struct transport2* tp[NUM_DATALINK_TP];
};

#define INITIALIZED_DATALINK_DEV { \
  .dev = NULL, \
  .tp[0 ... (NUM_DATALINK_TP-1)] = NULL \
}

struct RxDatalink{
  uint8_t rx_buf[MSG_SIZE];
  uint16_t rx_data_len;
  struct Datalink_device dl_dev[NUM_DATALINK_DEV];
};

#define INITIALIZED_DATALINK { \
  .rx_data_len = 0, \
  .dl_dev[0 ... (NUM_DATALINK_DEV-1)] = INITIALIZED_DATALINK_DEV \
}

extern struct RxDatalink datalink;


/** Check for new message and parse */
#define DlCheckAndParse() {   \
  if (dl_msg_available) {      \
    dl_parse_msg();            \
    dl_msg_available = FALSE;  \
  }                            \
}

/** Datalink kinds */
#define PPRZ 1
#define XBEE 2
#define UDP 3
#define SUPERBITRF 4


#if defined DATALINK && (DATALINK == PPRZ || DATALINK == XBEE)

#if DATALINK == PPRZ
#define UplinkDevice datalink_join(&dev_, PPRZ_UART)
#define UplinkTransport PprzTransport

#elif DATALINK == XBEE
#define UplinkDevice datalink_join(&dev_, XBEE_UART)
#define UplinkTransport XBeeTransport
#endif

static inline bool_t datalink_register(struct transport2* tp, void (*dl_parse)(const uint8_t *dl_msg, const uint16_t dl_msg_len)) {
  uint8_t dev_idx;
  uint8_t tp_idx;
  struct device* dev;
#ifdef _DATALINK_TRACES_
  bool_t result;
#endif

  //1- Get associated device
  dev = tp->api.rx_device(tp->tp_data);
  if(dev == NULL) { //transport has not been initialized yet
    DATALINK_TRACE("\tdatalink.h: register:  TRANSPORT %s HAS NOT AN ASSOCIATED DEVICE. EXECUTION ABORTED\n", tp->api.name(tp->tp_data));
    return FALSE;
  }

  //2- Register device (if possible) in datalink
  for (dev_idx = 0; dev_idx < NUM_DATALINK_DEV; dev_idx++) {
    if(datalink.dl_dev[dev_idx].dev == dev || datalink.dl_dev[dev_idx].dev == NULL) {
      datalink.dl_dev[dev_idx].dev = dev;
      break;
    }
  }
  if(dev_idx == NUM_DATALINK_DEV) { //No more devices can be registered
    DATALINK_TRACE("\tdatalink.h: register:  DATALINK DEVICE CANNOT BE STORED. EXECUTION ABORTED\n");
    return FALSE;
  }

  //3- Register transport (if possible) in datalink
  for (tp_idx = 0; tp_idx < NUM_DATALINK_TP; tp_idx++) {
    if(datalink.dl_dev[dev_idx].tp[tp_idx] == tp || datalink.dl_dev[dev_idx].tp[tp_idx] == NULL) {
      datalink.dl_dev[dev_idx].tp[tp_idx] = tp;
      break;
    }
  }
  if(tp_idx == NUM_DATALINK_TP) { //No more transports can be registered for the specified device
    DATALINK_TRACE("\tdatalink.h: register:  TRANSPORT %s CANNOT BE STORED. EXECUTION ABORTED\n", tp->api.name(tp->tp_data));
    return FALSE;
  }

  //4- Register callback (if possible) in transport
#ifdef _DATALINK_TRACES_
  result = tp->api.register_callback(tp->tp_data, dl_parse);
  if (result)
  { DATALINK_TRACE("\tdatalink.h: register:  DATALINK CORRECTLY REGISTERED (device = %s, transport = %s, callback = %p)\n", dev->api.name(dev->periph), tp->api.name(tp->tp_data), dl_parse); }
  else
  { DATALINK_TRACE("\tdatalink.h: register:  DATALINK CALLBACK CANNOT BE STORED. EXECUTION ABORTED\n"); }
  return result;
#else
  return tp->api.register_callback(tp->tp_data, dl_parse);
#endif
}

static inline void DatalinkEvent(void) {
  DATALINK_TRACE("\tdatalink.h: event:  HOLA\n");
  for(uint8_t i=0; (i < NUM_DATALINK_DEV && datalink.dl_dev[i].dev != NULL); i++) {
    struct device* dev = datalink.dl_dev[i].dev;
    //Check device and fill a local buffer.
    datalink.rx_data_len = 0;
    while(datalink.rx_data_len < MSG_SIZE && dev->api.char_available(dev->periph))
      datalink.rx_buf[datalink.rx_data_len++] = dev->api.getch(dev->periph);
    //Parse buffer through device associated transports
    for(uint8_t j = 0; (j < NUM_DATALINK_TP && datalink.dl_dev[i].tp[j] != NULL);j++) {
      struct transport2* tp = datalink.dl_dev[i].tp[j];
      tp->api.parse(tp->tp_data, datalink.rx_buf, datalink.rx_data_len);
    }
  }
}

#elif defined DATALINK && DATALINK == W5100

#define DatalinkEvent() {                       \
    W5100CheckAndParse(W5100, w5100_tp);        \
    DlCheckAndParse();                          \
  }

#elif defined DATALINK && DATALINK == UDP

#define DatalinkEvent() {                       \
    UdpCheckAndParse();                         \
    DlCheckAndParse();                          \
  }

#elif defined DATALINK && DATALINK == SUPERBITRF

#define DatalinkEvent() {                       \
    SuperbitRFCheckAndParse();                  \
    DlCheckAndParse();                          \
  }

#else
// Unknown DATALINK
#define DatalinkEvent() {}
#endif // DATALINK ==


// CALLBACK FUNCTIONS ------------------------------------------------------------------
extern void datalink_pprz_callback(const uint8_t*payload, const uint16_t payload_len);

extern void datalink_xbee_callback(const uint8_t*payload, const uint16_t payload_len);

#endif /* DATALINK_H */
