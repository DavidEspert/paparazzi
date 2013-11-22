#ifndef _DOWNLINK_HEADER_H_
#define _DOWNLINK_HEADER_H_

#include "std.h"
#include "generated/airframe.h" // AC_ID is required
#include "mcu_periph/uart.h"    // required for uart_transaction struct definition
#include "mcu_periph/dynamic_buffer.h"


// #define _DOWNLINK_SEND_DEBUG_

#ifdef _DOWNLINK_SEND_DEBUG_
#include <stdio.h> 
#define _DOWNLINK_SEND_TRACE_(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#else
#define _DOWNLINK_SEND_TRACE_(...)
#endif



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