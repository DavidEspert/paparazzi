#ifndef _DOWNLINK_HEADER_H_
#define _DOWNLINK_HEADER_H_

#include "std.h"
#include "generated/airframe.h" // AC_ID is required

typedef struct {
  const uint8_t ac_id;
  uint8_t msg_id;
}msgHeader;

static msgHeader msg_hd = { .ac_id = AC_ID};

#endif // _DOWNLINK_HEADER_H_