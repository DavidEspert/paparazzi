#include "transport_xbee.h"

struct xbee_transport_rx xbee_tp_rx = INITIALIZED_XBEE_DATA("XBEE_1");

//API functions declaration
void xbee_init(void* data, struct device* rx_dev);
struct device* xbee_rx_device(void* data);
char* xbee_name(void* data);
uint8_t xbee_header_len(void);
void xbee_header(uint8_t *buff, uint16_t msg_data_length);
uint8_t xbee_tail_len(void);
void xbee_tail(uint8_t *buff, uint16_t msg_data_length);
bool_t xbee_register_callback(void* data, void (*callback)(const uint8_t*, const uint16_t) );
void xbee_parse(void* data, uint8_t *c, uint16_t length);


//API functions definition
void xbee_init(void* t, struct device* rx_dev)                                  { XBeeTransport_init((struct xbee_transport_rx*) t, rx_dev); }
struct device* xbee_rx_device(void* data)                                       { return XBeeTransport_rx_device((struct xbee_transport_rx*) data); }
char* xbee_name(void* data)                                                     { return XBeeTransport_name((struct xbee_transport_rx*) data); }
uint8_t xbee_header_len(void)                                                   { return XBeeTransport_header_len(); }
void xbee_header(uint8_t *buff, uint16_t msg_data_length)                       { XBeeTransport_header(buff, msg_data_length); }
uint8_t xbee_tail_len(void)                                                     { return XBeeTransport_tail_len(); }
void xbee_tail(uint8_t *buff, uint16_t msg_data_length)                         { XBeeTransport_tail(buff, msg_data_length); }
bool_t xbee_register_callback(void* data, void (*callback)(const uint8_t*, const uint16_t) )
                                                                                { return XBeeTransport_register_callback((struct xbee_transport_rx*) t, callback); }
void xbee_parse(void* data, uint8_t *c, uint16_t length) {
  uint8_t* ptr = c;
  for (uint16_t i = 0; i < length; i++)
    XBeeTransport_parse((struct xbee_transport_rx*) data, *ptr++);
}

#define INITIALIZED_XBEE_API { \
  .init =               &xbee_init, \
  .rx_device =          &xbee_rx_device, \
  .name =               &xbee_name, \
  .header_len =         (3+XBEE_API_LEN), /*sizeof(struct xbee_header)*/ \
  .header =             &xbee_header, \
  .tail_len =           1, /*sizeof(struct xbee_tail)*/ \
  .tail =               &xbee_tail, \
  .register_callback =  &xbee_register_callback, \
  .parse =              &xbee_parse \
}

// XBEE_TP --------------------------------------------------------------------

//API struct (user will access to API through 'XBeeTransport')
struct transport2 XBeeTransport = {
  .data      =  &xbee_tp_rx,
  .api       =  INITIALIZED_XBEE_API
};

