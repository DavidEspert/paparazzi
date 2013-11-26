#include "transport_pprz.h"

struct pprz_transport_rx pprz_tp_rx;


//API functions declaration
void pprz_init(void);
uint8_t pprz_header_len(void);
void pprz_header(uint8_t *buff, uint16_t msg_data_length);
uint8_t pprz_tail_len(void);
void pprz_tail(uint8_t *buff, uint16_t msg_data_length);
void pprz_parse2(uint8_t c );

//API struct (user will access to API through 'PprzTransport')
struct DownlinkTransport PprzTransport = {
  .init =               &pprz_init,
  .header_len =         &pprz_header_len,
  .header =             &pprz_header,
  .tail_len =           &pprz_tail_len,
  .tail =               &pprz_tail,
  .parse =              &pprz_parse2
};


//API functions definition
void pprz_init(void)                                            { PprzTransport_init(); }
uint8_t pprz_header_len(void)                                   { return PprzTransport_header_len(); }
void pprz_header(uint8_t *buff, uint16_t msg_data_length)       { PprzTransport_header(buff, msg_data_length); }
uint8_t pprz_tail_len(void)                                     { return PprzTransport_tail_len(); }
void pprz_tail(uint8_t *buff, uint16_t msg_data_length)         { PprzTransport_tail(buff, msg_data_length); }
void pprz_parse2(uint8_t c)                                      { PprzTransport_parse(c); }


