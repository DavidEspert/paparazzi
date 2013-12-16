#include "transport_pprz.h"

// TX API ---------------------------------------------------------------------
//API functions declaration
uint8_t pprz_header_len(void);
void pprz_header(uint8_t *buff, uint16_t msg_data_length);
uint8_t pprz_tail_len(void);
void pprz_tail(uint8_t *buff, uint16_t msg_data_length);

//API functions definition
uint8_t pprz_header_len(void)                                                   { return PprzTransport_header_len(); }
void pprz_header(uint8_t *buff, uint16_t msg_data_length)                       { PprzTransport_header(buff, msg_data_length); }
uint8_t pprz_tail_len(void)                                                     { return PprzTransport_tail_len(); }
void pprz_tail(uint8_t *buff, uint16_t msg_data_length)                         { PprzTransport_tail(buff, msg_data_length); }

//API struct initialization
#define INITIALIZED_PPRZ_TX_API { \
  .header_len =         2, /*pprz_header_len*/ \
  .header =             &pprz_header, \
  .tail_len =           2, /*pprz_tail_len*/ \
  .tail =               &pprz_tail \
}

//API struct declaration
#if defined TRANSPORT_TX_1 && TRANSPORT_TX_1 == PPRZ
struct transport_tx     transport_tx_1 = { .api =  INITIALIZED_PPRZ_TX_API };
#elif defined TRANSPORT_TX_2 && TRANSPORT_TX_2 == PPRZ
struct transport_tx     transport_tx_2 = { .api =  INITIALIZED_PPRZ_TX_API };
#elif defined TRANSPORT_TX_3 && TRANSPORT_TX_3 == PPRZ
struct transport_tx     transport_tx_3 = { .api =  INITIALIZED_PPRZ_TX_API };
#endif


// struct transport_tx PprzTransport =  { .api = INITIALIZED_PPRZ_TX_API };




// RX API ---------------------------------------------------------------------
//API functions declaration
void pprz_init(void* data);
bool_t pprz_register_device(void* data, struct device* rx_dev);
struct device* pprz_rx_device(void* data);
char* pprz_name(void* data);
void pprz_parse(void* data, uint8_t byte);
bool_t pprz_message_received(void* data);
bool_t pprz_register_callback(void* data, void (*callback)(const uint8_t*, const uint16_t) );
void pprz_callback(void* data);

//API functions definition
void pprz_init(void* data)                                                      { PprzTransport_init((struct transport_rx_data_pprz*) data); }
bool_t pprz_register_device(void* data, struct device* rx_dev)                  { return PprzTransport_register_device((struct transport_rx_data_pprz*) data, rx_dev); }
struct device* pprz_rx_device(void* data)                                       { return PprzTransport_rx_device((struct transport_rx_data_pprz*) data); }
char* pprz_name(void* data)                                                     { return PprzTransport_name((struct transport_rx_data_pprz*) data); }
void pprz_parse(void* data, uint8_t byte)                                       { PprzTransport_parse((struct transport_rx_data_pprz*) data, byte); }
bool_t pprz_message_received(void* data)                                        { return PprzTransport_message_received((struct transport_rx_data_pprz*) data); }
bool_t pprz_register_callback(void* data, void (*callback)(const uint8_t*, const uint16_t) )
                                                                                { return PprzTransport_register_callback((struct transport_rx_data_pprz*) data, callback); }
void pprz_callback(void* data)                                                  { PprzTransport_callback((struct transport_rx_data_pprz*)data); }

//API struct initialization
#define INITIALIZED_PPRZ_RX_API { \
  .init =               &pprz_init, \
  .register_device =    &pprz_register_device, \
  .rx_device =          &pprz_rx_device, \
  .name =               &pprz_name, \
  .parse =              &pprz_parse, \
  .message_received =   &pprz_message_received, \
  .register_callback =  &pprz_register_callback, \
  .callback =           &pprz_callback \
}

//API struct declaration
#ifdef TRANSPORT_RX_PPRZ_1
struct transport_rx_data_pprz   tp_rx_data_pprz_1 = INITIALIZED_PPRZ_RX_DATA("PPRZ_1");
struct transport_rx transport_rx_PPRZ_1 = {
  .data      =  &tp_rx_data_pprz_1,
  .api       =  INITIALIZED_PPRZ_RX_API
};
#endif
#ifdef TRANSPORT_RX_PPRZ_2
struct transport_rx_data_pprz   tp_rx_data_pprz_2 = INITIALIZED_PPRZ_RX_DATA("PPRZ_2");
struct transport_rx transport_rx_PPRZ_2 = {
  .data      =  &tp_rx_data_pprz_2,
  .api       =  INITIALIZED_PPRZ_RX_API
};
#endif
