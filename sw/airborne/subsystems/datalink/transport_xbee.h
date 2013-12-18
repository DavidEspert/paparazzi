/*
 * Copyright (C) 2013  Guillem Jornet
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

/** \file subsystems/datalink/transport_xbee.h
 *  \brief XBee transport layer API
 * 
 *  This file provides a XBee transport layer interface. Functions included are:
 *  - init:       initializes XBee transport layer interface
 *  - header_len: returns XBee transport header size
 *  - header:     sets header in a given address
 *  - tail_len:   returns XBee transport tail size
 *  - header:     sets tail at the end of the message (header has to be set in advance)
 *  - parse:      processes a Rx byte
 * 
 *  This API can be accessed in two ways:
 *  1) Directly accessing through the functions described below
 *  2) Accessing through the 'struct DownlinkTransport XBeeTransport'.
 * 
 *  (The second option provides a way to exchange this transport layer between aplications)
 *
 */

#ifndef _DOWNLINK_TRANSPORT_XBEE_H_
#define _DOWNLINK_TRANSPORT_XBEE_H_

#include "subsystems/datalink/datalink.h"
#include "transport_xbee_config.h"
#include <string.h>     //required for memcpy

#ifdef XBEE868
#include "subsystems/datalink/xbee868.h"
#else /* Not 868 */
#include "subsystems/datalink/xbee24.h"
#endif
#include "subsystems/datalink/transport2.h"



// TX API ---------------------------------------------------------------------

/** HEADER & TAIL */

/** Constants for the API protocol */
#define XBEE_START 0x7e
#define TX_OPTIONS 0x00
#define NO_FRAME_ID 0

/** Ground station address */
#define GROUND_STATION_ADDR 0x100

#define XBEE_HEADER_LEN (3 + XBEE_API_LEN)
struct xbee_header{
  uint8_t  start;
  uint16_t len;
  uint8_t api[XBEE_API_LEN]; //Depends on XBEE868 or XBEE24
};
#define INITIALIZED_XBEE_HD(_msg_data_length) { \
  .start = XBEE_START, \
  .len = ((_msg_data_length) + XBEE_API_LEN), \
/*  .len = ((_msg_data_length) + XBEE_HEADER_LEN + XBEE_TAIL_LEN),*/ \
  INITIALIZED_XBEE_HD_API \
}

#define XBEE_TAIL_LEN 1
struct xbee_tail{
  uint8_t  cksum;
};

static inline uint8_t XBeeTransport_header_len(void){
  return XBEE_HEADER_LEN;
}

static inline void XBeeTransport_header(uint8_t *buff, uint16_t msg_data_length){
  uint8_t *ptr = buff;
  struct xbee_header hd = INITIALIZED_XBEE_HD(msg_data_length);
  
  *(ptr++) = *(&(hd.start));
  *(ptr++) = *((uint8_t*)(&hd.len) + 1);
  *(ptr++) = *(&(hd.len));
  memcpy(ptr, hd.api, XBEE_API_LEN);
}

static inline uint8_t XBeeTransport_tail_len(void){
  return XBEE_TAIL_LEN;
}

static inline void XBeeTransport_tail(uint8_t *buff, uint16_t msg_data_length){
  uint8_t cksum = 0;
  uint16_t i;
  uint16_t total_length = msg_data_length + XBEE_API_LEN;

  for(i = 3; i < (3 + total_length); i++){
    cksum += buff[i];
  }
  
  cksum = 0xff - cksum;
  buff[i] = cksum;
}

#if (defined TRANSPORT_TX_1 && TRANSPORT_TX_1 == XBEE) || \
    (defined TRANSPORT_TX_2 && TRANSPORT_TX_2 == XBEE)
extern struct transport_tx transport_tx_XBEE;
#endif


// RX API ---------------------------------------------------------------------

/** RX PARSING STATE MACHINE */
#define XBEE_UNINIT         0
#define XBEE_GOT_START      1
#define XBEE_GOT_LENGTH_MSB 2
#define XBEE_GOT_LENGTH_LSB 3
#define XBEE_GOT_PAYLOAD    4

struct transport_rx_data_xbee {
  // generic interface
  struct transport_rx_data_common common;
  // specific xbee transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t cs;
};
#define INITIALIZED_XBEE_RX_DATA(_name) { \
  .common = INITIALIZED_TP_RX_DATA_COMMON(_name), \
  .status = XBEE_UNINIT, \
  .payload_idx = 0, \
  .cs = 0 \
};

static inline void XBeeTransport_init(struct transport_rx_data_xbee* data) {
  transport_rx_data_common_init(&(data->common));
  data->status = XBEE_UNINIT;
  data->payload_idx = 0;
  data->cs = 0;
}

static inline bool_t XBeeTransport_register_device(struct transport_rx_data_xbee* data, struct device* rx_dev) {
  if(data->common.rx_dev == NULL) {
    data->common.rx_dev = rx_dev;
    //Config link
    xbee_config_link(rx_dev);
    return TRUE;
  }
  if(data->common.rx_dev == rx_dev) {
    return TRUE;
  }
  return FALSE;
}

static inline struct device* XBeeTransport_rx_device(struct transport_rx_data_xbee* data) {
  return (data->common.rx_dev);
}

static inline char* XBeeTransport_name(struct transport_rx_data_xbee* data) {
  return (data->common.name);
}

static inline void XBeeTransport_parse(struct transport_rx_data_xbee* data, uint8_t c) {
  switch (data->status) {
  case XBEE_UNINIT:
    if (c == XBEE_START)
      data->status++;
    break;
  case XBEE_GOT_START:
    if (data->common.msg_received) {
      data->common.ovrn++;
      TRANSPORT_TRACE("\ttransport_xbee: parse:  OVERUN!!!\n");
      goto error;
    }
    data->common.payload_len = c<<8;
    data->status++;
    break;
  case XBEE_GOT_LENGTH_MSB:
    data->common.payload_len |= c;
    data->status++;
    data->payload_idx = 0;
    data->cs=0;
    break;
  case XBEE_GOT_LENGTH_LSB:
    data->common.payload[data->payload_idx] = c;
    data->cs += c;
    data->payload_idx++;
    if (data->payload_idx == data->common.payload_len)
      data->status++;
    break;
  case XBEE_GOT_PAYLOAD:
    if (c + data->cs != 0xff)
      goto error;
    TRANSPORT_TRACE("\ttransport_xbee: parse:  XBEE MESSAGE FOUND BY %s\n", (data->common.name));
    TRANSPORT_PRINT_PAYLOAD(data->common);
    for(uint8_t i = 0; (i < TRANSPORT_NUM_CALLBACKS && data->common.callback[i] != NULL); i++)
      data->common.callback[i](data->common.payload, data->common.payload_len);
    data->common.msg_received = TRUE;
    goto restart;
    break;
  default:
    goto error;
  }
  return;
 error:
  data->common.error++;
 restart:
  data->status = XBEE_UNINIT;
  return;
}

static inline bool_t XBeeTransport_message_received(struct transport_rx_data_xbee* data) {
  return data->common.msg_received;
}

static inline bool_t XBeeTransport_register_callback(struct transport_rx_data_xbee* data, void (*callback)(const uint8_t*, const uint16_t) ) {
  for (uint8_t i = 0; i < TRANSPORT_NUM_CALLBACKS; i++) {
    if(data->common.callback[i] == callback || data->common.callback[i] == NULL) {
      data->common.callback[i] = callback;
      return TRUE;
    }
  }
  return FALSE;
}

static inline void XBeeTransport_callback(struct transport_rx_data_xbee* data) {
  if(data->common.msg_received) {
    for(uint8_t i = 0; i < TRANSPORT_NUM_CALLBACKS; i++)
      if(data->common.callback[i] != NULL)
        data->common.callback[i](data->common.payload, data->common.payload_len);
  }
  data->common.msg_received = FALSE;
}

#ifdef TRANSPORT_RX_XBEE_1
extern struct transport_rx transport_rx_XBEE_1;
#endif
#ifdef TRANSPORT_RX_XBEE_2
extern struct transport_rx transport_rx_XBEE_2;
#endif


#endif//_DOWNLINK_TRANSPORT_XBEE_H_
