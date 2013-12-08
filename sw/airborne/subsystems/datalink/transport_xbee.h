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
#include "generated/airframe.h" // AC_ID is required
#include <string.h>     //required for memcpy

#ifdef XBEE868
#include "subsystems/datalink/xbee868.h"
#else /* Not 868 */
#include "subsystems/datalink/xbee24.h"
#endif
#include "subsystems/datalink/transport2.h"



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
  .len = ((_msg_data_length) + sizeof(struct xbee_header) + sizeof(xbee_tail)), \
  INITIALIZED_XBEE_HD_API \
}

struct xbee_tail{
  uint8_t  cksum;
};


/** Initialisation in API mode and setting of the local address
 * FIXME: busy wait */
#define XBEE_MY_ADDR AC_ID

/** RX PARSING STATE MACHINE */

/** Status of the API packet receiver automata */
#define XBEE_UNINIT         0
#define XBEE_GOT_START      1
#define XBEE_GOT_LENGTH_MSB 2
#define XBEE_GOT_LENGTH_LSB 3
#define XBEE_GOT_PAYLOAD    4

struct xbee_transport_rx {
  // generic interface
  struct transport_data_common common;
  // specific xbee transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t cs;
};
#define INITIALIZED_XBEE_DATA(_name) { \
  .common = INITIALIZED_TP_DATA_COMMON(_name), \
  .status = XBEE_UNINIT, \
  .payload_idx = 0, \
  .cs = 0 \
};

extern struct xbee_transport_rx xbee_tp_rx;

#define AT_COMMAND_SEQUENCE "+++"
#define AT_SET_MY "ATMY"
#define AT_AP_MODE "ATAP1\r"
#define AT_EXIT "ATCN\r"

static inline uint8_t xbee_text_reply_is_ok(void) {
  char c[2];
  int count = 0;

  while (TransportLink(XBEE_UART,ChAvailable()))
  {
    char cc = TransportLink(XBEE_UART,Getch());
    if (count < 2)
      c[count] = cc;
    count++;
  }

  if ((count > 2) && (c[0] == 'O') && (c[1] == 'K'))
    return TRUE;

  return FALSE;
}

static inline uint8_t xbee_try_to_enter_api(void) {

  /** Switching to AT mode (FIXME: busy waiting) */
  XBeePrintString(XBEE_UART,AT_COMMAND_SEQUENCE);

  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  return xbee_text_reply_is_ok();
}

#define XBeeUartSetBaudrate(_a) TransportLink(XBEE_UART,SetBaudrate(_a))


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


// - XBEE TRANSPORT INTERFACE ----------------------------------------------------

static inline void XBeeTransport_init((struct pprz_transport_rx* data, struct device* rx_dev) {
  for (uint8_t i = 0; i < TRANSPORT_NUM_CALLBACKS; i++)
    data->common.callback[i] = NULL;
  data->common.rx_dev = rx_dev;
  data->status = XBEE_UNINIT;
//   data->common.msg_received = FALSE;

  // Empty buffer before init process
  while (TransportLink(XBEE_UART,ChAvailable()))
    TransportLink(XBEE_UART,Getch());

#ifndef NO_XBEE_API_INIT
  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  if (! xbee_try_to_enter_api() )
  {
    #ifdef XBEE_BAUD_ALTERNATE

      // Badly configured... try the alternate baudrate:
      XBeeUartSetBaudrate(XBEE_BAUD_ALTERNATE);
      if ( xbee_try_to_enter_api() )
      {
        // The alternate baudrate worked,
        XBeePrintString(XBEE_UART,XBEE_ATBD_CODE);
      }
      else
      {
        // Complete failure, none of the 2 baudrates result in any reply
        // TODO: set LED?

        // Set the default baudrate, just in case everything is right
        XBeeUartSetBaudrate(XBEE_BAUD);
        XBeePrintString(XBEE_UART,"\r");
      }

    #endif
    // Continue changing settings until the EXIT is issued.
  }

  /** Setting my address */
  XBeePrintString(XBEE_UART,AT_SET_MY);
  uint16_t addr = XBEE_MY_ADDR;
  XBeePrintHex16(XBEE_UART,addr);
  XBeePrintString(XBEE_UART,"\r");

  XBeePrintString(XBEE_UART,AT_AP_MODE);

#ifdef XBEE_INIT
  XBeePrintString(XBEE_UART,XBEE_INIT);
#endif

  /** Switching back to normal mode */
  XBeePrintString(XBEE_UART,AT_EXIT);

  XBeeUartSetBaudrate(XBEE_BAUD);

#endif
}

static inline struct device* XBeeTransport_rx_device(struct xbee_transport_rx* data) {
  return (data->common.rx_dev);
}

static inline char* XBeeTransport_name(struct xbee_transport_rx* data) {
  return (data->common.name);
}

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
  uint16_t total_length = msg_data_length + sizeof(struct xbee_api);

  for(uint16_t i = 3; i < (3 + total_length); i++){
    cksum += buff[i];
  }
  
  cksum = 0xff - cksum;
  buff[i] = cksum; //((struct xbee_tail *)(buff + sizeof(struct xbee_header) + msg_data_length))->cksum = cksum;
}

static inline bool_t XBeeTransport_register_callback(struct xbee_transport_rx* data, void (*callback)(const uint8_t*, const uint16_t) ) {
  for (uint8_t i = 0; i < TRANSPORT_NUM_CALLBACKS; i++) {
    if(data->common.callback[i] == callback || data->common.callback[i] == NULL) {
      data->common.callback[i] = callback;
      return TRUE;
    }
  }
  return FALSE;
}

static inline void XBeeTransport_parse(struct xbee_transport_rx* data, uint8_t c) {
  switch (data->status) {
  case XBEE_UNINIT:
    if (c == XBEE_START)
      data->status++;
    break;
  case XBEE_GOT_START:
//     if (data->common.msg_received) {
//       data->common.ovrn++;
//       goto error;
//     }
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
    for(uint8_t i = 0; (i < TRANSPORT_NUM_CALLBACKS && data->common.callback[i] != NULL); i++)
      data->common.callback[i](&(data->common.payload), data->common.payload_len);
//     data->common.msg_received = TRUE;
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


#define XBeePrintString(_dev, s) TransportLink(_dev,PrintString(s))
#define XBeePrintHex16(_dev, x) TransportLink(_dev,PrintHex16(x))


extern struct transport2 XBeeTransport;

#endif//_DOWNLINK_TRANSPORT_XBEE_H_
