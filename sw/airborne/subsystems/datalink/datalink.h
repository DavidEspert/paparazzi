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

#ifdef _DATALINK_TRACES_
#include <stdio.h>
#define DATALINK_TRACE(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#else
#define DATALINK_TRACE(...)
#endif

#include "std.h"
#include "dl_protocol.h"
//Available Devices
#include "subsystems/datalink/device_uart.h"
//Available Transports
#include "subsystems/datalink/transport_pprz.h"
#include "subsystems/datalink/transport_xbee.h"

EXTERN bool_t dl_msg_available;
/** Flag provided to control calls to ::dl_parse_msg. NOT used in this module*/

EXTERN uint16_t datalink_time;

#define MSG_SIZE 128
EXTERN uint8_t dl_buffer[MSG_SIZE]  __attribute__ ((aligned));

EXTERN void dl_parse_msg(void);
/** Should be called when chars are available in dl_buffer */


#define NUM_DATALINK_DEV                2
#define DATALINK_DEV_NOT_REGISTERED     NUM_DATALINK_DEV

extern struct device* datalink_dev[NUM_DATALINK_DEV];
#define INITIALIZED_DATALINK    { NULL }

#ifdef DATALINK

#define __ul_join(_y, _x) _y##_x
#define _ul_join(_y, _x) __ul_join(_y, _x)
#define ul_join(_chan, _fun) _ul_join(_chan, _fun)


// - Datalink transports --------------------------------------------------------------------------
#ifndef DATALINK_TRANSPORT
  #error "DATALINK is enabled but there is no DATALINK_TRANSPORT defined"
#else
  #define UplinkTransport       ul_join (transport_rx_, DATALINK_TRANSPORT)
  #if DATALINK_TRANSPORT == PPRZ_1 || DATALINK_TRANSPORT == PPRZ_2
    #define UplinkCallback      datalink_pprz_callback
  #elif DATALINK_TRANSPORT == XBEE_1 || DATALINK_TRANSPORT == XBEE_2
    #define UplinkCallback      datalink_xbee_callback
  #endif
#endif

// - Datalink devices -----------------------------------------------------------------------------
#ifdef DATALINK_SIM_DEVICE
   #undef DATALINK_DEVICE
   #define DATALINK_DEVICE DATALINK_SIM_DEVICE
#elif !defined(DATALINK_DEVICE)
   #error "DATALINK defined but not DATALINK_DEVICE. At least one datalink device is required"
#endif

#ifndef UplinkDevice
   #define UplinkDevice ul_join(dev_, DATALINK_DEVICE)
#endif



// - Datalink functions ---------------------------------------------------------------------------
/** Check for new message and parse */
#define DlCheckAndParse() {   \
  if (dl_msg_available) {      \
    dl_parse_msg();            \
    dl_msg_available = FALSE;  \
  }                            \
}


#if DATALINK == W5100

#define DatalinkEvent() {                       \
    W5100CheckAndParse(W5100, w5100_tp);        \
    DlCheckAndParse();                          \
  }

#elif DATALINK == UDP

#define DatalinkEvent() {                       \
    UdpCheckAndParse();                         \
    DlCheckAndParse();                          \
  }

#elif DATALINK == SUPERBITRF

#define DatalinkEvent() {                       \
    SuperbitRFCheckAndParse();                  \
    DlCheckAndParse();                          \
  }

#else

static inline bool_t datalink_register(struct transport_rx* tp_rx, struct device* dev, void (*callback)(const uint8_t *dl_msg, const uint16_t dl_msg_len)) {
  uint8_t dev_idx;

  //0- verify device and transport
  if(dev == NULL) {
    DATALINK_TRACE("\tdatalink.h: register:  NO DEVICE PROVIDED. EXECUTION ABORTED\n");
    return FALSE;
  }
  if(tp_rx == NULL) {
    DATALINK_TRACE("\tdatalink.h: register:  NO TRANSPORT PROVIDED. EXECUTION ABORTED\n");
    return FALSE;
  }

  //1- verify device <--> transport association
  {
    struct device* assoc_dev = tp_rx->api.rx_device(tp_rx->data);
    if(assoc_dev != NULL && assoc_dev != dev) {
      //Unacceptable: transport cannot parse data from different devices. Unregister and register again!
      DATALINK_TRACE("\tdatalink.h: register:  ATTEMPTING TO ASSOCIATE TRANSPORT %s WITH DEVICE %s BUT %s IS ALREADY ASSOCIATED WITH DEVICE %s. EXECUTION ABORTED\n", tp_rx->api.name(tp_rx->data), dev->api.name(dev->data), tp_rx->api.name(tp_rx->data), assoc_dev->api.name(assoc_dev->data));
      return FALSE;
    }
  }
  {
    struct transport_rx* assoc_tp = dev->api.rx_transport(dev->data);
    if(assoc_tp != NULL && assoc_tp != tp_rx) {
      //Maybe in future...
      DATALINK_TRACE("\tdatalink.h: register:  ATTEMPTING TO ASSOCIATE DEVICE %s WITH TRANSPORT %s BUT %s IS ALREADY ASSOCIATED WITH TRANSPORT %s. EXECUTION ABORTED\n", dev->api.name(dev->data), tp_rx->api.name(tp_rx->data), dev->api.name(dev->data), assoc_tp->api.name(assoc_tp->data));
      return FALSE;
    }
  }

  //2- register device (if possible) in datalink
  for (dev_idx = 0; dev_idx < NUM_DATALINK_DEV; dev_idx++) {
    if(datalink_dev[dev_idx] == dev || datalink_dev[dev_idx] == NULL) {
      datalink_dev[dev_idx] = dev;
      break;
    }
  }
  if(dev_idx == NUM_DATALINK_DEV) { //No more devices can be registered
    DATALINK_TRACE("\tdatalink.h: register:  DATALINK DEVICE CANNOT BE STORED. EXECUTION ABORTED\n");
    return FALSE;
  }

  //3 register callback (if possible) in transport
  if (tp_rx->api.register_callback(tp_rx->data, callback)) {
    //4- associate device <--> transport
    tp_rx->api.register_device(tp_rx->data, dev);     //this registration prevends against multiple transport association
    dev->api.register_transport(dev->data, tp_rx);    //this registration provides link for device 'check and parse'
    DATALINK_TRACE("\tdatalink.h: register:  DATALINK CORRECTLY REGISTERED (device = %s, transport = %s, callback = %p)\n", dev->api.name(dev->data), tp_rx->api.name(tp_rx->data), callback);
    return TRUE;
  }
  else {
    DATALINK_TRACE("\tdatalink.h: register:  DATALINK CALLBACK CANNOT BE STORED. EXECUTION ABORTED\n");
    return FALSE;
  }
}

static inline void DatalinkEvent(void) {
  for(uint8_t i=0; i < NUM_DATALINK_DEV; i++) {
    struct device* dev = datalink_dev[i];
    if(dev != NULL) {
//       DATALINK_TRACE("\tdatalink.h: event:  STARTING CHECK_AND_PARSE FROM DEVICE %s\n", dev->api.name(dev->data));
      //check and parse device data
      struct transport_rx* tp_rx = dev->api.rx_transport(dev->data);
      while (dev->api.byte_available(dev->data)) {
        uint8_t c = dev->api.get_byte(dev->data);
        //In future: if different TP per device allowed, parse data through all TPs
        //and callback if any message is found (stop parsing)
        tp_rx->api.parse(tp_rx->data, c);
        if(tp_rx->api.message_received(tp_rx->data))
          break; //goto callback;
      }
// callback:
      tp_rx->api.callback(tp_rx->data);
    }
  }
}

extern void datalink_pprz_callback(const uint8_t*payload, const uint16_t payload_len);

extern void datalink_xbee_callback(const uint8_t*payload, const uint16_t payload_len);

#endif // DATALINK == ...

#endif // DATALINK

#endif /* DATALINK_H */
