#include "transport_xbee.h"

// TX API ---------------------------------------------------------------------
//API functions declaration
uint8_t xbee_header_len(void);
void xbee_header(uint8_t *buff, uint16_t msg_data_length);
uint8_t xbee_tail_len(void);
void xbee_tail(uint8_t *buff, uint16_t msg_data_length);

//API functions declaration
uint8_t xbee_header_len(void)                                                   { return XBeeTransport_header_len(); }
void xbee_header(uint8_t *buff, uint16_t msg_data_length)                       { XBeeTransport_header(buff, msg_data_length); }
uint8_t xbee_tail_len(void)                                                     { return XBeeTransport_tail_len(); }
void xbee_tail(uint8_t *buff, uint16_t msg_data_length)                         { XBeeTransport_tail(buff, msg_data_length); }

//API struct initialization
#define INITIALIZED_XBEE_TX_API { \
  .header_len =         XBEE_HEADER_LEN, /*xbee_header_len*/ \
  .header =             &xbee_header, \
  .tail_len =           XBEE_TAIL_LEN, /*xbee_tail_len*/ \
  .tail =               &xbee_tail \
}

//API struct declaration
#if (defined TRANSPORT_TX_1 && TRANSPORT_TX_1 == XBEE) || \
    (defined TRANSPORT_TX_2 && TRANSPORT_TX_2 == XBEE)
struct transport_tx     transport_tx_XBEE = { .api       =  INITIALIZED_XBEE_TX_API };
#endif


// RX API ---------------------------------------------------------------------
//API functions declaration
void xbee_init(void* data);
bool_t xbee_register_device(void* data, struct device* rx_dev);
struct device* xbee_rx_device(void* data);
char* xbee_name(void* data);
void xbee_parse(void* data, uint8_t byte);
bool_t xbee_message_received(void* data);
bool_t xbee_register_callback(void* data, void (*callback)(const uint8_t*, const uint16_t) );
void xbee_callback(void* data);

//API functions definition
void xbee_init(void* data)                                                      { XBeeTransport_init((struct transport_rx_data_xbee*) data); }
bool_t xbee_register_device(void* data, struct device* rx_dev)                  { return XBeeTransport_register_device((struct transport_rx_data_xbee*) data, rx_dev); }
struct device* xbee_rx_device(void* data)                                       { return XBeeTransport_rx_device((struct transport_rx_data_xbee*) data); }
char* xbee_name(void* data)                                                     { return XBeeTransport_name((struct transport_rx_data_xbee*) data); }
void xbee_parse(void* data, uint8_t byte)                                       { XBeeTransport_parse((struct transport_rx_data_xbee*) data, byte); }
bool_t xbee_message_received(void* data)                                        { return XBeeTransport_message_received((struct transport_rx_data_xbee*) data); }
bool_t xbee_register_callback(void* data, void (*callback)(const uint8_t*, const uint16_t) )
                                                                                { return XBeeTransport_register_callback((struct transport_rx_data_xbee*) data, callback); }
void xbee_callback(void* data)                                                  { XBeeTransport_callback((struct transport_rx_data_xbee*) data); }
  //- auxiliar function for xbee initialization...
void xbee_init_callback(void *slot_p) {
  dynamic_buffer_free_slot_pointer(&dynamic_buff, (uint8_t*)slot_p);
}

//API struct initialization
#define INITIALIZED_XBEE_RX_API { \
  .init =               &xbee_init, \
  .register_device =    &xbee_register_device, \
  .rx_device =          &xbee_rx_device, \
  .name =               &xbee_name, \
  .parse =              &xbee_parse, \
  .message_received =   &xbee_message_received, \
  .register_callback =  &xbee_register_callback, \
  .callback =           &xbee_callback \
}

//API struct declaration
#ifdef TRANSPORT_RX_XBEE_1
struct transport_rx_data_xbee   tp_rx_data_xbee_1 = INITIALIZED_XBEE_RX_DATA("XBEE_1");
struct transport_rx transport_rx_XBEE_1 = {
  .data      =  &tp_rx_data_xbee_1,
  .api       =  INITIALIZED_XBEE_RX_API
};
#endif
#ifdef TRANSPORT_RX_XBEE_2
struct transport_rx_data_xbee   tp_rx_data_xbee_2 = INITIALIZED_XBEE_RX_DATA("XBEE_2");
struct transport_rx transport_rx_XBEE_2 = {
  .data      =  &tp_rx_data_xbee_2,
  .api       =  INITIALIZED_XBEE_RX_API
};
#endif

