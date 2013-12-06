/*
 * Copyright (C) 2013  Guillem Jornet
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "datalink.h"

struct RxDatalink datalink = INITIALIZED_DATALINK;


void datalink_pprz_callback(const uint8_t*payload, const uint16_t payload_len) {
  if(payload_len > MSG_SIZE) return;

  for(uint8_t i = 0; i < payload_len; i++)
    dl_buffer[i] = payload[i];
  dl_parse_msg();
}

#include "xbee.h"
void datalink_xbee_callback(const uint8_t*payload, const uint16_t payload_len) {
// uint8_t xbee_rssi;

  if(payload_len > MSG_SIZE) return;

  switch (payload[0]) {
  case XBEE_RX_ID:
  case XBEE_TX_ID: /* Useful if A/C is connected to the PC with a cable */
//     XbeeGetRSSI(payload);
    for(uint8_t i = XBEE_RFDATA_OFFSET; i < payload_len; i++)
      dl_buffer[i-XBEE_RFDATA_OFFSET] = payload[i];
    dl_parse_msg();
    break;
  default:
    return;
  }
}


