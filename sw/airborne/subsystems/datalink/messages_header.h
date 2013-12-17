#ifndef _DOWNLINK_HEADER_H_
#define _DOWNLINK_HEADER_H_

#include "std.h"
#include "generated/airframe.h" // AC_ID is required
#include "mcu_periph/dynamic_buffer.h"


#ifdef _DOWNLINK_SEND_TRACES_
#include <stdio.h> 
#define _DOWNLINK_SEND_TRACE_(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#else
#define _DOWNLINK_SEND_TRACE_(...)
#endif

// #define DOWNLINK_PUT_1_BYTE(_x)  *(ptr++) = *( (uint8_t*)(_x) )
// #define DOWNLINK_PUT_2_BYTE(_x)  DOWNLINK_PUT_1_BYTE(_x); DOWNLINK_PUT_1_BYTE(_x+1)
// #define DOWNLINK_PUT_4_BYTE(_x)  DOWNLINK_PUT_2_BYTE(_x); DOWNLINK_PUT_2_BYTE(_x+2)
// #ifdef __IEEE_BIG_ENDIAN /* From machine/ieeefp.h */
// #define DOWNLINK_PUT_8_BYTE(_x)  DOWNLINK_PUT_4_BYTE(_x+4); DOWNLINK_PUT_4_BYTE(_x)
// #else
// #define DOWNLINK_PUT_8_BYTE(_x)  DOWNLINK_PUT_4_BYTE(_x); DOWNLINK_PUT_4_BYTE(_x+4)
// #endif

struct msgHeader {
  const uint8_t ac_id;
  uint8_t msg_id;
};
static struct msgHeader msg_hd = { .ac_id = AC_ID};
#define MSG_HD_LEN sizeof(struct msgHeader)


static void message_callback(void *slot_p) {
  dynamic_buffer_free_slot_pointer(&dynamic_buff, (uint8_t*)slot_p);
}

#endif // _DOWNLINK_HEADER_H_