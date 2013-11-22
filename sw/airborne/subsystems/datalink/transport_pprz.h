#ifndef _DOWNLINK_TRANSPORT_PPRZ_H_
#define _DOWNLINK_TRANSPORT_PPRZ_H_

#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/transport2.h"
// #include "generated/airframe.h" // AC_ID is required

/** TX DEFINES & STRUCTS */
#define STX_PPRZ_TX  0x99

struct pprz_header{
  uint8_t       stx;
  uint8_t       length;
};

struct pprz_tail{
  uint8_t       ck_a;
  uint8_t       ck_b;
};

/** RX DEFINES & STRUCTS */

// PPRZ parsing state machine
#define UNINIT      0
#define GOT_STX     1
#define GOT_LENGTH  2
#define GOT_PAYLOAD 3
#define GOT_CRC1    4

struct pprz_transport_rx {
  // generic interface
  struct PayloadTransport trans;
  // specific pprz transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t ck_a, ck_b;
};

extern struct pprz_transport_rx pprz_tp_rx;

// - PAPARAZZI TRANSPORT INTERFACE ----------------------------------------------------

static inline uint8_t PprzTransport_header_len(void){
// return sizeof(struct pprz_header);  
  return 2;
}

static inline void PprzTransport_header(uint8_t *buff, uint8_t msg_data_length){
  // 'header.length' is the length of 'transport+message'. This is
  // {STX, len, MESSAGE, CK_A, CK_B}
  
  //set header
  buff[0] = STX_PPRZ_TX;                //((struct pprz_header *)buff)->stx = STX_PPRZ_TX;
  buff[1] = msg_data_length + 4;        //((struct pprz_header *)buff)->length = 2+msg_data_length+2;
}

static inline uint8_t PprzTransport_tail_len(void){
//  return sizeof(struct pprz_tail);
  return 2;
}

static inline void PprzTransport_tail(uint8_t *buff, uint8_t msg_data_length){
  // Calculate data checksum
  struct pprz_tail tl;
  int i;

  tl.ck_a = buff[1];
  tl.ck_b = buff[1];

  for(i = 2; i < (2 + msg_data_length); i++){
    tl.ck_a += buff[i];
    tl.ck_b += tl.ck_a;
  }
  
  buff[i++] = tl.ck_a; //((struct pprz_tail *)(buff + sizeof(struct header) + msg_data_length))->ck_a = tl.ck_a;
  buff[i]   = tl.ck_b; //((struct pprz_tail *)(buff + sizeof(struct header) + msg_data_length))->ck_b = tl.ck_b;
}

static inline void PprzTransport_parse(uint8_t c) {
  switch (pprz_tp_rx.status) {
  case UNINIT:
    if (c == STX_PPRZ_TX)
      pprz_tp_rx.status++;
    break;
  case GOT_STX:
    if (pprz_tp_rx.trans.msg_received) {
      pprz_tp_rx.trans.ovrn++;
      goto error;
    }
    pprz_tp_rx.trans.payload_len = c-4; // Counting STX, LENGTH and CRC1 and CRC2
    pprz_tp_rx.ck_a = pprz_tp_rx.ck_b = c;
    pprz_tp_rx.status++;
    pprz_tp_rx.payload_idx = 0;
    break;
  case GOT_LENGTH:
    pprz_tp_rx.trans.payload[pprz_tp_rx.payload_idx] = c;
    pprz_tp_rx.ck_a += c; pprz_tp_rx.ck_b += pprz_tp_rx.ck_a;
    pprz_tp_rx.payload_idx++;
    if (pprz_tp_rx.payload_idx == pprz_tp_rx.trans.payload_len)
      pprz_tp_rx.status++;
    break;
  case GOT_PAYLOAD:
    if (c != pprz_tp_rx.ck_a)
      goto error;
    pprz_tp_rx.status++;
    break;
  case GOT_CRC1:
    if (c != pprz_tp_rx.ck_b)
      goto error;
    pprz_tp_rx.trans.msg_received = TRUE;
    goto restart;
  default:
    goto error;
  }
  return;
 error:
  pprz_tp_rx.trans.error++;
 restart:
  pprz_tp_rx.status = UNINIT;
  return;
}

static inline void PprzTransport_parse_payload(void) {
/*  uint8_t i;
  for(i = 0; i < pprz_tp_rx.trans.payload_len; i++)
    dl_buffer[i] = pprz_tp_rx.trans.payload[i];
  dl_msg_available = TRUE;*/
}

//NON INLINE FUNCTIONS
extern uint8_t pprz_header_len(void);
extern void pprz_header(uint8_t *buff, uint8_t msg_data_length);
extern uint8_t pprz_tail_len(void);
extern void pprz_tail(uint8_t *buff, uint8_t msg_data_length);
extern void pprz_parse2(uint8_t c );
extern void pprz_parse_payload2(void);

extern struct DownlinkTransport PprzTransport;

#endif//_DOWNLINK_TRANSPORT_PPRZ_H_
