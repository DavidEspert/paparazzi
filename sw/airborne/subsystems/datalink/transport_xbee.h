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

struct xbee_header{
  uint8_t  start;
  uint16_t len;
  uint8_t api[XBEE_API_LEN]; //Depends on XBEE868 or XBEE24
};
#define INITIALIZED_XBEE_HD(_msg_data_length) { \
  .start = XBEE_START, \
  .len = ((_msg_data_length) + sizeof(struct xbee_header) + sizeof(struct xbee_tail)), \
  INITIALIZED_XBEE_HD_API \
}

struct xbee_tail{
  uint8_t  cksum;
};

static inline uint8_t XBeeTransport_header_len(void){
  return sizeof(struct xbee_header);
}

static inline void XBeeTransport_header(uint8_t *buff, uint16_t msg_data_length){
  struct xbee_header hd = INITIALIZED_XBEE_HD(msg_data_length);
  memcpy(buff, &hd, sizeof(struct xbee_header));
}

static inline uint8_t XBeeTransport_tail_len(void){
//   return sizeof(struct xbee_tail);
  return 1;
}

static inline void XBeeTransport_tail(uint8_t *buff, uint16_t msg_data_length){
  uint8_t cksum = 0;
  uint16_t i;
  uint16_t total_length = msg_data_length + XBEE_API_LEN;

  for(i = 3; i < (3 + total_length); i++){
    cksum += buff[i];
  }
  
  cksum = 0xff - cksum;
  buff[i] = cksum; //((struct xbee_tail *)(buff + sizeof(struct xbee_header) + msg_data_length))->cksum = cksum;
}

// #ifdef TX_TRANSPORT_1 && TX_TRANSPORT_1 == PPRZ
// extern struct transport_tx transport_tx_1;
// #elif defined TX_TRANSPORT_2 && TX_TRANSPORT_2 == PPRZ
// extern struct transport_tx transport_tx_2;
// #elif defined TX_TRANSPORT_3 && TX_TRANSPORT_3 == PPRZ
// extern struct transport_tx transport_tx_3;
// #endif



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



// - auxiliar functions for initialzation (baudrate config) ------------------
#include "generated/airframe.h" // AC_ID is required

#define XBEE_MY_ADDR AC_ID

#define AT_COMMAND_SEQUENCE "+++"
#define AT_SET_MY "ATMY"
#define AT_AP_MODE "ATAP1\r"
#define AT_EXIT "ATCN\r"

/** Initialisation in API mode and setting of the local address
 * FIXME: busy wait */
#if XBEE_BAUD == B9600
    #define XBEE_BAUD_ALTERNATE B57600
    #define XBEE_ATBD_CODE "ATBD3\rATWR\r"
    #pragma message "Experimental: XBEE-API@9k6 auto-baudrate 57k6 -> 9k6 (stop ground link for correct operation)"
#elif XBEE_BAUD == B57600
    #define XBEE_BAUD_ALTERNATE B9600
    #define XBEE_ATBD_CODE "ATBD6\rATWR\r"
    #pragma message "Experimental: XBEE-API@57k6 auto-baudrate 9k6 -> 57k6 (stop ground link for correct operation)"
#else
    #warning XBEE-API Non default baudrate: auto-baud disabled
#endif

extern void xbee_init_callback(void *slot_p);

#define XBEE_CONFIG_PRIORITY 0
static inline void XBeeTransport_send_data(struct device* rx_dev, uint8_t *data, uint8_t data_len) {
  uint8_t dev_slot;
  uint8_t buff_slot;
  uint8_t ta_len =    rx_dev->api.transaction_len;

  /* 1.- try to get a device's 'transaction' slot */
  if(rx_dev->api.check_free_space(rx_dev->data, &dev_slot)){
    /* 2.- try to get a slot in dynamic buffer */
    if(dynamic_buffer_check_free_space(&dynamic_buff, (ta_len + data_len), &buff_slot)){
      /* 3.- get buffer pointer */
      uint8_t *buff = dynamic_buffer_get_slot_pointer(&dynamic_buff, buff_slot);

      /* SET TRANSACTION: CONTAINS DATA POINTER, LENGTH AND CALLBACK */
      /* 4.- set transaction in buffer */
      rx_dev->api.transaction_pack(buff, (buff + ta_len), (data_len), NULL, 0, &xbee_init_callback);

      /* SET DATA */
      /* 5.- set data in buffer */
      memcpy((buff + ta_len), data, data_len);

      /* SUMMIT TRANSACTION */
      /* 6.- send data */
      rx_dev->api.transaction_summit(rx_dev->data, dev_slot, buff, XBEE_CONFIG_PRIORITY);
    }
    else {
      /* 7.- release device's slot */
      rx_dev->api.free_space(rx_dev->data, dev_slot);
    }
  }
}

static inline uint8_t XBeeTransport_print_string(char* s, uint8_t* buff) {
  uint8_t i = 0;
  while (s[i]) {
    buff[i] = (uint8_t) (s[i]);
    i++;
  }
  return i;
}

static inline uint8_t XBeeTransport_print_hex(uint8_t* data, uint8_t data_len, uint8_t* buff) {
  const uint8_t hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
//   uint8_t high;
//   uint8_t low;

  for (uint8_t i = 0; i < data_len; i++) {
//     high = (data[i] & 0xF0)>>4;
//     low  = data[i] & 0x0F;
    buff[(2*i)] =   hex[ ((data[i] & 0xF0)>>4) ];
    buff[(2*i+1)] = hex[  (data[i] & 0x0F)     ];
  }
  return (2*data_len); // CAUTION: if data_len >= 128 output will be truncated.
}

static inline uint8_t xbee_text_reply_is_ok(struct device* rx_dev) {
  char c[2];
  int count = 0;

//   while (TransportLink(XBEE_UART,ChAvailable()))
  while (rx_dev->api.char_available(rx_dev->data))
  {
    char cc = rx_dev->api.getch(rx_dev->data);
    if (count < 2)
      c[count] = cc;
    count++;
  }

  if ((count > 2) && (c[0] == 'O') && (c[1] == 'K'))
    return TRUE;

  return FALSE;
}

static inline uint8_t xbee_try_to_enter_api(struct device* rx_dev) {

  /** Switching to AT mode (FIXME: busy waiting) */
//   XBeePrintString(XBEE_UART,AT_COMMAND_SEQUENCE);

//   uint8_t buff[16];
//   uint8_t command_len;
  char command_seq[4] = AT_COMMAND_SEQUENCE;
//   command_len = XBeeTransport_print_string(command_seq, buff);
//   XBeeTransport_send_data(rx_dev, buff, command_len);
  XBeeTransport_send_data(rx_dev, (uint8_t*) command_seq, 3);

  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  return xbee_text_reply_is_ok(rx_dev);
}

static inline void xbee_config_link(struct transport_rx_data_xbee* data, struct device* dev) {
  uint8_t buff[128];
  uint16_t offset = 0;
  char eol[3] = "\r";

  // Empty buffer before init process
//   while (TransportLink(XBEE_UART,ChAvailable()))
//     TransportLink(XBEE_UART,Getch());
  while(rx_dev->api.char_available(rx_dev->data))
    rx_dev->api.getch(rx_dev->data);

#ifndef NO_XBEE_API_INIT
  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  if (! xbee_try_to_enter_api(rx_dev) )
  {
    #ifdef XBEE_BAUD_ALTERNATE

      // Badly configured... try the alternate baudrate:
//       XBeeUartSetBaudrate(XBEE_BAUD_ALTERNATE);
      rx_dev->api.set_baudrate(rx_dev->data, XBEE_BAUD_ALTERNATE);
      if ( xbee_try_to_enter_api(rx_dev) )
      {
        // The alternate baudrate worked,
//         XBeePrintString(XBEE_UART,XBEE_ATBD_CODE);
        char atbd[14] = XBEE_ATBD_CODE;
        offset += XBeeTransport_print_string(atbd, &buff[offset]);
      }
      else
      {
        // Complete failure, none of the 2 baudrates result in any reply
        // TODO: set LED?

        // Set the default baudrate, just in case everything is right
//         XBeeUartSetBaudrate(XBEE_BAUD);
//         XBeePrintString(XBEE_UART,"\r");
        rx_dev->api.set_baudrate(rx_dev->data, XBEE_BAUD);
        offset += XBeeTransport_print_string(eol, &buff[offset]);
      }

    #endif
    // Continue changing settings until the EXIT is issued.
  }

  /** Setting my address */
//   XBeePrintString(XBEE_UART,AT_SET_MY);
//   uint16_t addr = XBEE_MY_ADDR;
//   XBeePrintHex16(XBEE_UART,addr);
//   XBeePrintString(XBEE_UART,"\r");
// 
//   XBeePrintString(XBEE_UART,AT_AP_MODE);

  char set_my[5] = AT_SET_MY;
  uint16_t addr = XBEE_MY_ADDR;
  char ap_mode[8] = AT_AP_MODE;
  offset += XBeeTransport_print_string(set_my, &buff[offset]);
  offset += XBeeTransport_print_hex((uint8_t*) &addr, 2, &buff[offset]);
  offset += XBeeTransport_print_string(eol, &buff[offset]);

  offset += XBeeTransport_print_string(ap_mode, &buff[offset]);

#ifdef XBEE_INIT
//   XBeePrintString(XBEE_UART,XBEE_INIT);
  char init[32] = XBEE_INIT;
  offset += XBeeTransport_print_string(init, &buff[offset]);
#endif

  /** Switching back to normal mode */
//   XBeePrintString(XBEE_UART,AT_EXIT);
  char exit[7] = AT_EXIT;
  offset += XBeeTransport_print_string(exit, &buff[offset]);

  XBeeTransport_send_data(rx_dev, buff, offset);

//   XBeeUartSetBaudrate(XBEE_BAUD);
  sys_time_usleep(0025000); //wait 0.025s before switch back (data has to be transmitted)
  rx_dev->api.set_baudrate(rx_dev->data, XBEE_BAUD);

#endif
}
// #define XBeeUartSetBaudrate(_a) TransportLink(XBEE_UART,SetBaudrate(_a))
// #define XBeePrintString(_dev, s) TransportLink(_dev,PrintString(s))
// #define XBeePrintHex16(_dev, x) TransportLink(_dev,PrintHex16(x))

// - End of auxiliar functions ------------------------------------------------


static inline void XBeeTransport_init(struct transport_rx_data_xbee* data, struct device* rx_dev) {
  transport_rx_data_common_init(&(data->common));
  data->status = XBEE_UNINIT;
  data->payload_idx = 0;
  data->cs = 0;
  //Config link
  xbee_config_link(data, rx_dev);
}

static inline bool_t XBeeTransport_register_device(struct transport_rx_data_xbee* data, struct device* rx_dev) {
  if(data->common.rx_dev == NULL || data->common.rx_dev == rx_dev) {
    data->common.rx_dev = rx_dev;
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
    TRANSPORT_TRACE("\ttransport_xbee: parse:  PPRZ MESSAGE FOUND BY %s\n", (data->common.name));
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

static inline bool_t XBeeTransport_message_received(void* data) {
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

// #ifdef RX_TRANSPORT_1 && RX_TRANSPORT_1 == XBEE
// extern struct transport_rx transport_rx_1;
// #endif
// #ifdef RX_TRANSPORT_2 && RX_TRANSPORT_2 == XBEE
// extern struct transport_rx transport_rx_2;
// #endif

#endif//_DOWNLINK_TRANSPORT_XBEE_H_
