#include "transport_xbee.h"

struct xbee_transport_rx xbee_tp_rx;


//API functions declaration
void xbee_init(void);
uint8_t xbee_header_len(void);
void xbee_header(uint8_t *buff, uint16_t msg_data_length);
uint8_t xbee_tail_len(void);
void xbee_tail(uint8_t *buff, uint16_t msg_data_length);
void xbee_parse(uint8_t c );

//API struct (user will access to API through 'XBeeTransport')
struct DownlinkTransport XBeeTransport = {
  .init =               &xbee_init,
  .header_len =         &xbee_header_len,
  .header =             &xbee_header,
  .tail_len =           &xbee_tail_len,
  .tail =               &xbee_tail,
  .parse =              &xbee_parse
};


//API functions definition
void xbee_init(void)                                            { XBeeTransport_init(); }
uint8_t xbee_header_len(void)                                   { return XBeeTransport_header_len(); }
void xbee_header(uint8_t *buff, uint16_t msg_data_length)       { XBeeTransport_header(buff, msg_data_length); }
uint8_t xbee_tail_len(void)                                     { return XBeeTransport_tail_len(); }
void xbee_tail(uint8_t *buff, uint16_t msg_data_length)         { XBeeTransport_tail(buff, msg_data_length); }
void xbee_parse(uint8_t c)                                      { XBeeTransport_parse(c); }


