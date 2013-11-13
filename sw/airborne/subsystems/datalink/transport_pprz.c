#include <stdarg.h>
#include "transport2.h"
//#include "generated/airframe.h" // AC_ID is required

#define STX_PPRZ_TX  0x99

#define PPRZ_TRANS_FALSE 0
#define PPRZ_TRANS_TRUE 1

// PPRZ parsing state machine
#define UNINIT      0
#define GOT_STX     1
#define GOT_LENGTH  2
#define GOT_PAYLOAD 3
#define GOT_CRC1    4
//static uint8_t status = UNINIT;

//Structs definition
struct header{
  uint8_t       stx;
  uint8_t       length;
  //uint8_t     timestamp;
//  uint8_t       ac_id;
//  uint8_t       msg_id;
};

struct tail{
  uint8_t	ck_a;
  uint8_t	ck_b;
};


//Transport functions declaration
uint8_t pprz_header_len(void);
void pprz_header(uint8_t *buff, uint8_t msg_data_length);
uint8_t pprz_tail_len(void);
void pprz_tail(uint8_t *buff, uint8_t msg_data_length);
void pprz_parse(void);


//Public transport struct: API
/*static*/struct DownlinkTransport PprzTransport = {
  .header_len = &pprz_header_len,
  .header =     &pprz_header,
  .tail_len =   &pprz_tail_len,
  .tail =       &pprz_tail,
  .parse =      &pprz_parse,
  .msg_received = PPRZ_TRANS_FALSE
};
/*static struct DownlinkTransport PprzTransport = { pprz_header_len, pprz_header, pprz_tail_len, pprz_tail};*/

/*struct DownlinkTransport XBeeTransport = {
  .header_len = &pprz_header_len,
  .header =     &pprz_header,
  .tail_len =   &pprz_tail_len,
  .tail =       &pprz_tail,
  .parse =      &pprz_parse,
  .msg_received = PPRZ_TRANS_FALSE
};*/

/*struct DownlinkTransport IvyTransport = {
  .header_len =	&pprz_header_len,
  .header =     &pprz_header,
  .tail_len =   &pprz_tail_len,
  .tail =       &pprz_tail,
  .parse =      &pprz_parse,
  .msg_received = PPRZ_TRANS_FALSE
};*/




uint8_t pprz_header_len(void){
// return sizeof(struct header);  
  return 2;
}

void pprz_header(uint8_t *buff, uint8_t msg_data_length){
  // 'header.length' is the length of frame under chksum analysis. This is
  // frame = {STX, len, MESSAGE, CK_A, CK_B}
  
  //set header
  buff[0] = STX_PPRZ_TX;                //((struct header *)buff)->stx = STX_PPRZ_TX;
  buff[1] = msg_data_length + 4;        //((struct header *)buff)->length = msg_data_length+2;
}

uint8_t pprz_tail_len(void){
//  return sizeof(struct tail);
  return 2;
}

void pprz_tail(uint8_t *buff, uint8_t msg_data_length){
  // Calculate data checksum
  struct tail tl;
  int i;

  tl.ck_a = buff[1];
  tl.ck_b = buff[1];

  for(i = 2; i < (2 + msg_data_length); i++){
    tl.ck_a += buff[i];
    tl.ck_b += tl.ck_a;
  }
  
  buff[i++] = tl.ck_a; //((struct tail *)(buff + sizeof(struct header) + msg_data_length))->ck_a = tl.ck_a;
  buff[i]   = tl.ck_b; //((struct tail *)(buff + sizeof(struct header) + msg_data_length))->ck_b = tl.ck_b;
}

void pprz_parse(void) {
/*
  if(c != STX_PPRZ_TX) return;
  
  if(PprzTransport.msg_received){
    PprzTransport.ovrn++;
    PprzTransport.error++;
    return;
  }

  switch (t->status) {
  case UNINIT:
    if (c == STX)
      t->status++;
    break;
  case GOT_STX:
    if (t->trans.msg_received) {
      t->trans.ovrn++;
      goto error;
    }
    t->trans.payload_len = c-4; // Counting STX, LENGTH and CRC1 and CRC2
    t->ck_a = t->ck_b = c;
    t->status++;
    t->payload_idx = 0;
    break;
  case GOT_LENGTH:
    t->trans.payload[t->payload_idx] = c;
    t->ck_a += c; t->ck_b += t->ck_a;
    t->payload_idx++;
    if (t->payload_idx == t->trans.payload_len)
      t->status++;
    break;
  case GOT_PAYLOAD:
    if (c != t->ck_a)
      goto error;
    t->status++;
    break;
  case GOT_CRC1:
    if (c != t->ck_b)
      goto error;
    t->trans.msg_received = TRUE;
    goto restart;
  default:
    goto error;
  }
  return;
 error:
  t->trans.error++;
 restart:
  t->status = UNINIT;
  return;*/
}

