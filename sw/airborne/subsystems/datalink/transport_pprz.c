#include "transport_pprz.h"

struct pprz_transport_rx pprz_tp_rx;


//NON INLINE FUNCTIONS
uint8_t pprz_header_len(void)                                   { return PprzTransport_header_len(); }
void pprz_header(uint8_t *buff, uint8_t msg_data_length)        { PprzTransport_header(buff, msg_data_length); }
uint8_t pprz_tail_len(void)                                     { return PprzTransport_tail_len(); }
void pprz_tail(uint8_t *buff, uint8_t msg_data_length)          { PprzTransport_tail(buff, msg_data_length); }
void pprz_parse2(uint8_t c)                                      { PprzTransport_parse(c); }
void pprz_parse_payload2(void)                                   { PprzTransport_parse_payload(); }


//Public transport struct: API
struct DownlinkTransport PprzTransport = {
  .header_len =         &pprz_header_len,
  .header =             &pprz_header,
  .tail_len =           &pprz_tail_len,
  .tail =               &pprz_tail,
  .parse =              &pprz_parse2,
  .parse_payload =      &pprz_parse_payload2
};

