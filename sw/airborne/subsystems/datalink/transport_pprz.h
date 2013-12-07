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

#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/transport2.h"
// #include "generated/airframe.h" // AC_ID is required


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



/** RX PARSING STATE MACHINE */

/** Status of the API packet receiver automata */
#define UNINIT      0
#define GOT_STX     1
#define GOT_LENGTH  2
#define GOT_PAYLOAD 3
#define GOT_CRC1    4

struct pprz_transport_rx {
  // generic interface
  struct transport_data trans;
  // specific pprz transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t ck_a, ck_b;
};

extern struct pprz_transport_rx pprz_tp_rx;

// static inline void PprzTransport_parse_payload(void) {
//   uint8_t i;
//   for(i = 0; i < pprz_tp_rx.trans.payload_len; i++)
//     dl_buffer[i] = pprz_tp_rx.trans.payload[i];
//   dl_msg_available = TRUE;
// }


// - PAPARAZZI TRANSPORT INTERFACE ----------------------------------------------------

static inline void PprzTransport_init(struct pprz_transport_rx* t, struct device* rx_dev) {
  for (uint8_t i = 0; i < TRANSPORT_NUM_CALLBACKS; i++)
    t->trans.callback[i] = NULL;
  t->trans.rx_dev = rx_dev;
  t->status = UNINIT;
}

static inline struct device* PprzTransport_rx_device(struct pprz_transport_rx* t) {
  return (t->trans.rx_dev);
}

static inline char* PprzTransport_name(struct pprz_transport_rx* t) {
  return (t->trans.name);
}

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

static inline bool_t PprzTransport_register_callback(struct pprz_transport_rx* t, void (*callback)(const uint8_t*, const uint16_t) ) {
  for (uint8_t i = 0; i < TRANSPORT_NUM_CALLBACKS; i++) {
    if(t->trans.callback[i] == callback || t->trans.callback[i] == NULL) {
      t->trans.callback[i] = callback;
      return TRUE;
    }
  }
  return FALSE;
}

static inline void PprzTransport_parse(struct pprz_transport_rx* t, uint8_t c) {
  switch (t->status) {
  case UNINIT:
    if (c == STX_PPRZ_TX)
      t->status++;
    break;
  case GOT_STX:
//     if (t->trans.msg_received) {
//       t->trans.ovrn++;
//       goto error;
//     }
    t->trans.payload_len = c-4; // Counting STX, LENGTH and CRC1 and CRC2
    t->ck_a = t->ck_b = c;
    t->status++;
    t->payload_idx = 0;
    break;
  case GOT_LENGTH:
    t->trans.payload[t->payload_idx] = c;
    t->ck_a += c; t->ck_b += t->ck_a;
    t->payload_idx++;
    if (t->payload_idx == t->trans.payload_len)
      t->status++;
    break;
  case GOT_PAYLOAD:
    if (c != t->ck_a)
      goto error;
    t->status++;
    break;
  case GOT_CRC1:
    if (c != t->ck_b)
      goto error;
    TRANSPORT_TRACE("\ttransport_pprz: PprzTransport_parse:  PPRZ MESSAGE FOUND BY %s\n", (t->trans.name));
    for(uint8_t i = 0; (i < TRANSPORT_NUM_CALLBACKS && t->trans.callback[i] != NULL); i++)
      t->trans.callback[i](t->trans.payload, t->trans.payload_len);
//     t->trans.msg_received = TRUE;
    goto restart;
  default:
    goto error;
  }
  return;
 error:
  t->trans.error++;
 restart:
  t->status = UNINIT;
  return;
}

// #define PprzBuffer(_dev) TransportLink(_dev,ChAvailable())
// #define ReadPprzBuffer(_dev,_trans) { while (TransportLink(_dev,ChAvailable())&&!(_trans.trans.msg_received)) parse_pprz(&(_trans),TransportLink(_dev,Getch())); }
/*#define PprzCheckAndParse(_dev,_trans) {  \
  if (PprzBuffer(_dev)) {                 \
    ReadPprzBuffer(_dev,_trans);          \
    if (_trans.trans.msg_received) {      \
      pprz_parse_payload(&(_trans));      \
      _trans.trans.msg_received = FALSE;  \
    }                                     \
  }                                       \
}*/

extern struct transport2 PprzTransport;

#endif// _TRANSPORT_PPRZ_H_
