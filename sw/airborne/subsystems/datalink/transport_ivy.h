#ifndef _DOWNLINK_TRANSPORT_IVY_H_
#define _DOWNLINK_TRANSPORT_IVY_H_

#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/transport2.h"
// #include "generated/airframe.h" // AC_ID is required

/** TX DEFINES & STRUCTS */

struct ivy_header{
};

struct ivy_tail{
};

/** RX DEFINES & STRUCTS */

struct ivy_transport_rx {
  // generic interface
  struct PayloadTransport trans;
  // specific ivy transport variables
};

extern struct ivy_transport_rx  ivy_tp_rx;



// - IVY TRANSPORT INTERFACE ----------------------------------------------------

static inline uint8_t IvyTransport_header_len(void){
  return 0;
}

static inline void IvyTransport_header(uint8_t *buff, uint8_t msg_data_length){
}

static inline uint8_t IvyTransport_tail_len(void){
  return 0;
}

static inline void IvyTransport_tail(uint8_t *buff, uint8_t msg_data_length){
}

static inline void IvyTransport_parse(uint8_t c) {
}

static inline void IvyTransport_parse_payload(void) {
}

//NON INLINE FUNCTIONS
extern uint8_t ivy_header_len(void);
extern void ivy_header(uint8_t *buff, uint8_t msg_data_length);
extern uint8_t ivy_tail_len(void);
extern void ivy_tail(uint8_t *buff, uint8_t msg_data_length);
extern void ivy_parse(uint8_t c );
extern void ivy_parse_payload(void);

extern struct DownlinkTransport IvyTransport;

#endif//_DOWNLINK_TRANSPORT_IVY_H_
