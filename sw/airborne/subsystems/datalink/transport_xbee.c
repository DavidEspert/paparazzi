#include "transport_xbee.h"

struct xbee_transport_rx xbee_tp_rx;


//NON INLINE FUNCTIONS
uint8_t xbee_header_len(void)                                   { return XBeeTransport_header_len(); }
void xbee_header(uint8_t *buff, uint8_t msg_data_length)        { XBeeTransport_header(buff, msg_data_length); }
uint8_t xbee_tail_len(void)                                     { return XBeeTransport_tail_len(); }
void xbee_tail(uint8_t *buff, uint8_t msg_data_length)          { XBeeTransport_tail(buff, msg_data_length); }
void xbee_parse(uint8_t c)                                      { XBeeTransport_parse(c); }
void xbee_parse_payload(void)                                   { XBeeTransport_parse_payload(); }


//Public transport struct: API
struct DownlinkTransport XBeeTransport = {
  .header_len =         &xbee_header_len,
  .header =             &xbee_header,
  .tail_len =           &xbee_tail_len,
  .tail =               &xbee_tail,
  .parse =              &xbee_parse,
  .parse_payload =      &xbee_parse_payload
};

