/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/jevois.c"
 * @author Gautier Hattenberger
 * Decoder for standardized messages from the JEVOIS smart camera (http://jevois.org)
 */

#include "modules/computer_vision/jevois.h"

#include "std.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"
#include "math/pprz_algebra_float.h"

// max string length
#define JEVOIS_MAX_LEN 32
// max number of coordinates
#define JEVOIS_MAX_COORD 18
// temp buffer size

// generic JEVOIS message structure
struct jevois_msg_t {
  uint8_t type;
  char id[JEVOIS_MAX_LEN];
  uint8_t nb;
  int16_t coord[JEVOIS_MAX_COORD];
  uint16_t dim[3];
  struct FloatQuat quat;
  char extra[JEVOIS_MAX_LEN];
};

// decoder state
enum jevois_state {
  JV_SYNC = 0,
  JV_TYPE,
  JV_ID,
  JV_SIZE,
  JV_COORD,
  JV_DIM,
  JV_QUAT,
  JV_EXTRA,
  JV_SEND_MSG
};

// jevois struct
struct jevois_t {
  enum jevois_state state; // decoder state
  char buf[JEVOIS_MAX_LEN]; // temp buffer
  uint8_t idx; // temp index
  struct jevois_msg_t msg; // last decoded message
};

struct jevois_t jevois;

// initialization
void jevois_init(void)
{
  jevois.state = JV_SYNC;
  jevois.idx = 0;
  memset(jevois.buf, 0, JEVOIS_MAX_LEN);
}

// parsing function
static void jevois_parse(struct jevois_t *jv, char c)
{
  switch (jv->state) {
    case JV_SYNC:
      // wait for sync (newline character)
      if (c == '\n') {
        jv->state = JV_TYPE;
        jv->idx = 0;
      }
      break;
    case JV_TYPE:
      if (c == ' ') {
        break; // in case skip white spaces
      }
      jv->buf[jv->idx++] = c;
      // parse type
      // TODO set nb when possible
      // FIXME test each case individualy
      if (jv->idx > 1) {
        if (jv->buf[0] == 'T') {
          jv->state = JV_COORD;
          jv->msg.type = 0;
        } else if (jv->buf[0] == 'N') {
          jv->state = JV_ID;
          jv->msg.type = 1;
        } else if (jv->buf[0] == 'D') {
          jv->state = JV_ID;
          jv->msg.type = 2;
        } else if (jv->buf[0] == 'F') {
          jv->state = JV_ID;
          jv->msg.type = 3;
        } else {
          jv->state = JV_SYNC;
        }
        if (jv->buf[1] == '1') {
          jv->msg.type += 10;
        } else if (jv->buf[1] == '2') {
          jv->msg.type += 20;
        } else if (jv->buf[1] == '3') {
          jv->msg.type += 30;
        } else if (jv->buf[1] == '4') {
          jv->msg.type += 40;
        } else {
          jv->state = JV_SYNC;
        }
        jv->idx = 0;
      }
      break;
    case JV_ID:
      jv->msg.id[jv->idx++] = c;
      if (c == '\0') {
        if (jv->msg.type == JEVOIS_MSG_F2 ||
            jv->msg.type == JEVOIS_MSG_F3) {
          jv->state = JV_SIZE; // parse n before coordinates
        } else {
          jv->state = JV_COORD; // parse directly coordinates
        }
        break;
      }
      if (jv->idx >= JEVOIS_MAX_LEN) {
        jv->state = JV_TYPE;
      }
      break;
    case JV_SIZE:
      // TODO
      break;
    case JV_COORD:
      // TODO
      break;
    case JV_DIM:
      // TODO
      break;
    case JV_QUAT:
      // TODO
      break;
    case JV_EXTRA:
      // TODO
      break;
    case JV_SEND_MSG:
      // TODO
      break;
    default:
      // error, back to SYNC
      jv->state = JV_SYNC;
      break;
  }
}


// UART polling function
void jevois_event(void)
{
  // Look for data on serial link and send to parser
  while (uart_char_available(&JEVOIS_DEV)) {
    uint8_t ch = uart_getch(&JEVOIS_DEV);
    jevois_parse(&jevois, ch);
  }
}


