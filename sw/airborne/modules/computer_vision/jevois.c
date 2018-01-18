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
  uint8_t idx; // temp buffer index
  uint8_t n; // temp coordinates/dimension index
  struct jevois_msg_t msg; // last decoded message
};

struct jevois_t jevois;

// initialization
void jevois_init(void)
{
  jevois.state = JV_SYNC;
  jevois.idx = 0;
  jevois.n = 0;
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
        jv->n = 0;
      }
      break;
    case JV_TYPE:
      if (c == ' ') {
        break; // in case skip white spaces
      }
      jv->buf[jv->idx++] = c; // fill buffer
      // parse type
      if (jv->idx > 1) {
        if (jv->buf[0] == 'T' && jv->buf[1] == '1') {
          jv->state = JV_COORD;
          jv->msg.type = JEVOIS_MSG_T1;
          jv->msg.nb = 1;
        } else if (jv->buf[0] == 'N' && jv->buf[1] == '1') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_N1;
          jv->msg.nb = 1;
        } else if (jv->buf[0] == 'D' && jv->buf[1] == '1') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_D1;
          jv->msg.nb = 2;
        } else if (jv->buf[0] == 'T' && jv->buf[1] == '2') {
          jv->state = JV_COORD;
          jv->msg.type = JEVOIS_MSG_T2;
          jv->msg.nb = 2;
        } else if (jv->buf[0] == 'N' && jv->buf[1] == '2') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_N2;
          jv->msg.nb = 2;
        } else if (jv->buf[0] == 'D' && jv->buf[1] == '2') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_D2;
          jv->msg.nb = 8;
        } else if (jv->buf[0] == 'F' && jv->buf[1] == '2') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_F2;
          jv->msg.nb = 0;
        } else if (jv->buf[0] == 'T' && jv->buf[1] == '3') {
          jv->state = JV_COORD;
          jv->msg.type = JEVOIS_MSG_T3;
          jv->msg.nb = 3;
        } else if (jv->buf[0] == 'N' && jv->buf[1] == '3') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_N3;
          jv->msg.nb = 3;
        } else if (jv->buf[0] == 'D' && jv->buf[1] == '3') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_D3;
          jv->msg.nb = 3;
        } else if (jv->buf[0] == 'F' && jv->buf[1] == '3') {
          jv->state = JV_ID;
          jv->msg.type = JEVOIS_MSG_F3;
          jv->msg.nb = 0;
        } else {
          jv->state = JV_SYNC; // error
        }
        jv->idx = 0;
      }
      break;
    case JV_ID:
      if (c == ' ' && jv->idx == 0) {
        jv->idx++; // skip first white space
      }
      else if (c == ' ' && jv->idx > 0) {
        jv->buf[jv->idx] = '\0'; // end string
        if (jv->msg.type == JEVOIS_MSG_F2 ||
            jv->msg.type == JEVOIS_MSG_F3) {
          jv->state = JV_SIZE; // parse n before coordinates
        } else {
          jv->state = JV_COORD; // parse directly coordinates
        }
        break;
      }
      else {
        jv->msg.id[jv->idx++] = c;
        if (jv->idx > JEVOIS_MAX_LEN - 1) {
          jv->state = JV_SYNC; // too long, return to sync
        }
      }
      break;
    case JV_SIZE:
      if (c == ' ') {
        jv->buf[jv->idx] = '\0'; // end string
        jv->msg.nb = (uint8_t)atoi(jv->buf); // store size
        jv->state = JV_COORD;
        jv->idx = 0;
      }
      else {
        jv->buf[jv->idx++] = c; // fill buffer
      }
      break;
    case JV_COORD:
      if (c == ' ' && jv->idx == 0) {
        break;
      }
      else if ((c == ' ' || c == '\n' || c == '\r') && jv->idx > 0) {
        jv->buf[jv->idx] = '\0'; // end string
        jv->msg.coord[jv->n++] = (int16_t)atoi(jv->buf); // store value
        if (jv->n == jv->msg.nb) {
          // got all coordinates, go to next state
          jv->n = 0; // reset number of received elements
          jv->idx = 0; // reset index
          switch (jv->msg.type) {
            case JEVOIS_MSG_T1:
            case JEVOIS_MSG_T2:
            case JEVOIS_MSG_T3:
              jv->state = JV_SEND_MSG;
              break;
            case JEVOIS_MSG_N1:
            case JEVOIS_MSG_N2:
            case JEVOIS_MSG_N3:
            case JEVOIS_MSG_D3:
              jv->state = JV_DIM;
              break;
            case JEVOIS_MSG_D1:
            case JEVOIS_MSG_D2:
            case JEVOIS_MSG_F2:
            case JEVOIS_MSG_F3:
              jv->state = JV_EXTRA;
              break;
            default:
              jv->state = JV_SYNC; // error
              break;
          }
        }
        jv->idx = 0; // reset index
      }
      else {
        jv->buf[jv->idx++] = c; // fill buffer
      }
      break;
    case JV_DIM:
      if (c == ' ') {
        jv->buf[jv->idx] = '\0'; // end string
        jv->msg.dim[jv->n++] = (uint16_t)atoi(jv->buf); // store dimension
        if (jv->n == jv->msg.nb) {
          // got all dimensions, go to next state
          jv->n = 0; // reset number of received elements
          jv->idx = 0; // reset index
          if (jv->msg.type == JEVOIS_MSG_D3) {
            jv->state = JV_QUAT;
          } else {
            jv->state = JV_SEND_MSG;
          }
          break;
        }
        jv->idx = 0; // reset index
      }
      else {
        jv->buf[jv->idx++] = c; // fill buffer
      }
      break;
    case JV_QUAT:
      if (c == ' ') {
        jv->buf[jv->idx] = '\0';
        float q = 0.f;//(float) atof(jv->buf);
        switch (jv->n) {
          case 0:
            jv->msg.quat.qi = q; // TODO check quaternion order
            break;
          case 1:
            jv->msg.quat.qx = q;
            break;
          case 2:
            jv->msg.quat.qy = q;
            break;
          case 3:
            jv->msg.quat.qz = q;
            break;
          case 4:
            jv->state = JV_EXTRA;
            break;
          default:
            jv->state = JV_SYNC; // error
            break;
        }
        jv->n++;
        jv->idx = 0; // reset index
      }
      else {
        jv->buf[jv->idx++] = c; // fill buffer
      }
      break;
    case JV_EXTRA:
      if (c == ' ') {
        jv->buf[jv->idx] = '\0'; // end string
        jv->state = JV_SEND_MSG;
      }
      else {
        jv->msg.id[jv->idx++] = c; // store extra string
        if (jv->idx > JEVOIS_MAX_LEN - 1) {
          jv->state = JV_SYNC; // too long, return to sync
        }
      }
      break;
    case JV_SEND_MSG:
      // send ABI message
      AbiSendMsgJEVOIS_MSG(CV_JEVOIS_ID,
          jv->msg.type,
          jv->msg.id,
          jv->msg.nb,
          jv->msg.coord,
          jv->msg.dim,
          jv->msg.quat,
          jv->msg.extra);
      jv->state = JV_SYNC;
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
  while (uart_char_available(&(JEVOIS_DEV))) {
    uint8_t ch = uart_getch(&(JEVOIS_DEV));
    jevois_parse(&jevois, ch);
  }
}


