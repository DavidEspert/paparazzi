#include "transport_pprz.h"

struct pprz_transport_rx pprz_tp_rx;

//API functions declaration
void pprz_init(void);
uint8_t pprz_header_len(void);
void pprz_header(uint8_t *buff, uint16_t msg_data_length);
uint8_t pprz_tail_len(void);
void pprz_tail(uint8_t *buff, uint16_t msg_data_length);
void pprz_parse(void* tp_data, uint8_t *c, uint16_t length);


//API functions definition
void pprz_init(void)                                            { PprzTransport_init(); }
uint8_t pprz_header_len(void)                                   { return PprzTransport_header_len(); }
void pprz_header(uint8_t *buff, uint16_t msg_data_length)       { PprzTransport_header(buff, msg_data_length); }
uint8_t pprz_tail_len(void)                                     { return PprzTransport_tail_len(); }
void pprz_tail(uint8_t *buff, uint16_t msg_data_length)         { PprzTransport_tail(buff, msg_data_length); }
void pprz_parse(void* tp_data, uint8_t *c, uint16_t length) {
  uint8_t* ptr = c;
  for (uint16_t i = 0; i < length; i++)
    PprzTransport_parse((struct pprz_transport_rx*) tp_data, *ptr++);
}

#define INITIALIZED_PPRZ_API { \
  .init =               &pprz_init, \
  .header_len =         2, /*sizeof(struct pprz_header)*/ \
  .header =             &pprz_header, \
  .tail_len =           2, /*sizeof(struct pprz_tail)*/ \
  .tail =               &pprz_tail, \
  .parse =              &pprz_parse \
}

// PPRZ_TP --------------------------------------------------------------------

//API struct (user will access to API through 'PprzTransport')
struct transport2 PprzTransport = {
  .tp_data   =  &pprz_tp_rx,
  .api       =  INITIALIZED_PPRZ_API
};

