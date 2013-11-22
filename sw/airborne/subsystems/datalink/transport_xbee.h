#ifndef _DOWNLINK_TRANSPORT_XBEE_H_
#define _DOWNLINK_TRANSPORT_XBEE_H_

#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/transport2.h"
// #include "generated/airframe.h" // AC_ID is required

/** TX DEFINES & STRUCTS */

struct xbee_header{
};

struct xbee_tail{
};


/** RX DEFINES & STRUCTS */

/** Constants for the API protocol */
#define XBEE_START 0x7e
#define TX_OPTIONS 0x00
#define NO_FRAME_ID 0


/** Ground station address */
#define GROUND_STATION_ADDR 0x100

/** Initialisation in API mode and setting of the local address
 * FIXME: busy wait */
#define XBEE_MY_ADDR AC_ID

// XBEE parsing state machine

// uint8_t xbee_cs;
// uint8_t xbee_rssi;

#define AT_COMMAND_SEQUENCE "+++"
#define AT_SET_MY "ATMY"
#define AT_AP_MODE "ATAP1\r"
#define AT_EXIT "ATCN\r"

// PPRZ parsing state machine

struct xbee_transport_rx {
  // generic interface
  struct PayloadTransport trans;
  // specific xbee transport variables
};

extern struct xbee_transport_rx xbee_tp_rx;



// - XBEE TRANSPORT INTERFACE ----------------------------------------------------

static inline uint8_t XBeeTransport_header_len(void){
  return 0;
}

static inline void XBeeTransport_header(uint8_t *buff, uint8_t msg_data_length){
}

static inline uint8_t XBeeTransport_tail_len(void){
  return 0;
}

static inline void XBeeTransport_tail(uint8_t *buff, uint8_t msg_data_length){
}

static inline void XBeeTransport_parse(uint8_t c) {
}

static inline void XBeeTransport_parse_payload(void) {
}

//NON INLINE FUNCTIONS
extern uint8_t xbee_header_len(void);
extern void xbee_header(uint8_t *buff, uint8_t msg_data_length);
extern uint8_t xbee_tail_len(void);
extern void xbee_tail(uint8_t *buff, uint8_t msg_data_length);
extern void xbee_parse(uint8_t c );
extern void xbee_parse_payload(void);

extern struct DownlinkTransport XBeeTransport;

#endif//_DOWNLINK_TRANSPORT_XBEE_H_
