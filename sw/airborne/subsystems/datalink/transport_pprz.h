/** \file subsystems/datalink/transport_pprz.h
 *  \brief Paparazzi transport layer API
 * 
 *  This file provides a Paparazzi transport layer interface. Functions included are:
 *  - init:       initializes Paparazzi transport layer interface
 *  - header_len: returns Paparazzi transport header size
 *  - header:     sets header in a given address
 *  - tail_len:   returns Paparazzi transport tail size
 *  - header:     sets tail at the end of the message (header has to be set in advance)
 *  - parse:      processes a Rx byte
 * 
 *  This API can be accessed in two ways:
 *  1) Directly accessing through the functions described below
 *  2) Accessing through the 'struct DownlinkTransport PprzTransport'.
 * 
 *  (The second option provides a way to exchange this transport layer between aplications)
 *
 */
#ifndef _TRANSPORT_PPRZ_H_
#define _TRANSPORT_PPRZ_H_

// #include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/transport2.h"
// #include "generated/airframe.h" // AC_ID is required


// TX API ---------------------------------------------------------------------

/** HEADER & TAIL */
#define STX_PPRZ_TX  0x99
struct pprz_header{
  uint8_t       stx;
  uint8_t       length;
};

struct pprz_tail{
  uint8_t       ck_a;
  uint8_t       ck_b;
};

static inline uint8_t PprzTransport_header_len(void){
// return sizeof(struct pprz_header);  
  return 2;
}

static inline void PprzTransport_header(uint8_t *buff, uint16_t msg_data_length){
  // 'header.length' is the length of 'transport+message'. This is
  // {STX, len, MESSAGE, CK_A, CK_B}
  
  //set header
  buff[0] = STX_PPRZ_TX;                //((struct pprz_header *)buff)->stx = STX_PPRZ_TX;
  if(msg_data_length < 252)
    buff[1] = msg_data_length + 4;      //((struct pprz_header *)buff)->length = 2+msg_data_length+2;
  else
    buff[1] = 4;                        //rx parse will reject the message
}

static inline uint8_t PprzTransport_tail_len(void){
//  return sizeof(struct pprz_tail);
  return 2;
}

static inline void PprzTransport_tail(uint8_t *buff, uint16_t msg_data_length){
  // Calculate data checksum
  struct pprz_tail tl;
  int i;

  tl.ck_a = buff[1];
  tl.ck_b = buff[1];

  for(i = 2; i < (2 + msg_data_length); i++){
    tl.ck_a += buff[i];
    tl.ck_b += tl.ck_a;
  }
  
  buff[i++] = tl.ck_a; //((struct pprz_tail *)(buff + sizeof(struct pprz_header) + msg_data_length))->ck_a = tl.ck_a;
  buff[i]   = tl.ck_b; //((struct pprz_tail *)(buff + sizeof(struct pprz_header) + msg_data_length))->ck_b = tl.ck_b;
}

// #if defined TRANSPORT_TX_1 && TRANSPORT_TX_1 == PPRZ
// extern struct transport_tx transport_tx_1;
// #elif defined TRANSPORT_TX_2 && TRANSPORT_TX_2 == PPRZ
// extern struct transport_tx transport_tx_2;
// #endif


// RX API ---------------------------------------------------------------------


/** RX PARSING STATE MACHINE */
#define UNINIT      0
#define GOT_STX     1
#define GOT_LENGTH  2
#define GOT_PAYLOAD 3
#define GOT_CRC1    4

struct transport_rx_data_pprz {
  // generic interface
  struct transport_rx_data_common common;
  // specific pprz transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t ck_a, ck_b;
};
#define INITIALIZED_PPRZ_RX_DATA(_name) { \
  .common = INITIALIZED_TP_RX_DATA_COMMON(_name), \
  .status = UNINIT, \
  .payload_idx = 0, \
  .ck_a = 0, \
  .ck_b = 0 \
};

static inline void PprzTransport_init(struct transport_rx_data_pprz* data) {
  transport_rx_data_common_init(&(data->common));
  data->status = UNINIT;
  data->payload_idx = 0;
  data->ck_a = 0;
  data->ck_b = 0;
}

static inline bool_t PprzTransport_register_device(struct transport_rx_data_pprz* data, struct device* rx_dev) {
  if(data->common.rx_dev == NULL || data->common.rx_dev == rx_dev) {
    data->common.rx_dev = rx_dev;
    return TRUE;
  }
  return FALSE;
}

static inline struct device* PprzTransport_rx_device(struct transport_rx_data_pprz* data) {
  return (data->common.rx_dev);
}

static inline char* PprzTransport_name(struct transport_rx_data_pprz* data) {
  return (data->common.name);
}

static inline void PprzTransport_parse(struct transport_rx_data_pprz* data, uint8_t c) {
  switch (data->status) {
  case UNINIT:
    if (c == STX_PPRZ_TX)
      data->status++;
    break;
  case GOT_STX:
    if (data->common.msg_received) {
      data->common.ovrn++;
      goto error;
    }
    data->common.payload_len = c-4; // Counting STX, LENGTH and CRC1 and CRC2
    data->ck_a = data->ck_b = c;
    data->status++;
    data->payload_idx = 0;
    break;
  case GOT_LENGTH:
    data->common.payload[data->payload_idx] = c;
    data->ck_a += c; data->ck_b += data->ck_a;
    data->payload_idx++;
    if (data->payload_idx == data->common.payload_len)
      data->status++;
    break;
  case GOT_PAYLOAD:
    if (c != data->ck_a)
      goto error;
    data->status++;
    break;
  case GOT_CRC1:
    if (c != data->ck_b)
      goto error;
    TRANSPORT_TRACE("\ttransport_pprz: parse:  PPRZ MESSAGE FOUND BY %s\n", (data->common.name));
    TRANSPORT_PRINT_PAYLOAD(data->common);
    data->common.msg_received = TRUE;
    goto restart;
  default:
    goto error;
  }
  return;
 error:
  data->common.error++;
 restart:
  data->status = UNINIT;
  return;
}

static inline bool_t PprzTransport_message_received(struct transport_rx_data_pprz* data) {
  return data->common.msg_received;
}

static inline bool_t PprzTransport_register_callback(struct transport_rx_data_pprz* data, void (*callback)(const uint8_t*, const uint16_t) ) {
  for (uint8_t i = 0; i < TRANSPORT_NUM_CALLBACKS; i++) {
    if(data->common.callback[i] == callback || data->common.callback[i] == NULL) {
      data->common.callback[i] = callback;
      return TRUE;
    }
  }
  return FALSE;
}

static inline void PprzTransport_callback(struct transport_rx_data_pprz* data) {
  if(data->common.msg_received) {
    for(uint8_t i = 0; i < TRANSPORT_NUM_CALLBACKS; i++)
      if(data->common.callback[i] != NULL)
        data->common.callback[i](data->common.payload, data->common.payload_len);
  }
  data->common.msg_received = FALSE;
}

#ifdef TRANSPORT_RX_PPRZ_1
extern struct transport_rx transport_rx_PPRZ_1;
#endif
#ifdef TRANSPORT_RX_PPRZ_2
extern struct transport_rx transport_rx_PPRZ_2;
#endif

#endif// _TRANSPORT_PPRZ_H_
