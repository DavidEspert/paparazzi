#include "transport_xbee.h"

struct xbee_transport_rx xbee_tp_rx;

//API functions declaration
void xbee_init(void);
uint8_t xbee_header_len(void);
void xbee_header(uint8_t *buff, uint16_t msg_data_length);
uint8_t xbee_tail_len(void);
void xbee_tail(uint8_t *buff, uint16_t msg_data_length);
void xbee_parse(void* tp_data, uint8_t *c, uint16_t length);


//API functions definition
void xbee_init(void)                                            { XBeeTransport_init(); }
uint8_t xbee_header_len(void)                                   { return XBeeTransport_header_len(); }
void xbee_header(uint8_t *buff, uint16_t msg_data_length)       { XBeeTransport_header(buff, msg_data_length); }
uint8_t xbee_tail_len(void)                                     { return XBeeTransport_tail_len(); }
void xbee_tail(uint8_t *buff, uint16_t msg_data_length)         { XBeeTransport_tail(buff, msg_data_length); }
void xbee_parse(void* tp_data, uint8_t *c, uint16_t length) {
  uint8_t* ptr = c;
  for (uint16_t i = 0; i < length; i++)
    XBeeTransport_parse((struct xbee_transport_rx*) tp_data, *ptr++);
}

#define INITIALIZED_XBEE_API { \
  .init =               &xbee_init, \
  .header_len =         (3+XBEE_API_LEN), /*sizeof(struct xbee_header)*/ \
  .header =             &xbee_header, \
  .tail_len =           1, /*sizeof(struct xbee_tail)*/ \
  .tail =               &xbee_tail, \
  .parse =              &xbee_parse \
}

// XBEE_TP --------------------------------------------------------------------

//API struct (user will access to API through 'XBeeTransport')
struct transport2 XBeeTransport = {
  .tp_data   =  &xbee_tp_rx,
  .api       =  INITIALIZED_XBEE_API
};

