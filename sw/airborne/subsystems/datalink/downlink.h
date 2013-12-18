/*
 * Copyright (C) 2003-2006  Pascal Brisset, Antoine Drouin
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
 *
 */

/** \file downlink.h
 *  \brief Common code for AP and FBW telemetry
 *
 */

#ifndef DOWNLINK_H
#define DOWNLINK_H

#include <inttypes.h>

#include "generated/modules.h"
//Available Devices
#include "subsystems/datalink/device_uart.h"
//Available Transports
#include "subsystems/datalink/transport_pprz.h"
#include "subsystems/datalink/transport_xbee.h"



#if defined SITL /* Software In The Loop (Simulation) */

#if defined (DOWNLINK_SIM_DEVICE) && DOWNLINK_SIM_DEVICE == SIM_UART
#include "subsystems/datalink/simUart.h"
#undef  DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_SIM_DEVICE
#else
/** Software In The Loop simulation uses IVY bus directly as the transport layer */
#include "ivy_transport.h"
#endif

#else /** SITL */
// #include "subsystems/datalink/udp.h"
// #include "subsystems/datalink/w5100.h"
#if USE_SUPERBITRF
#include "subsystems/datalink/superbitrf.h"
#endif
#if USE_AUDIO_TELEMETRY
#include "subsystems/datalink/audio_telemetry.h"
#endif
#ifdef USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif

#endif /** !SITL */




#define __dl_join(_y, _x) _y##_x
#define _dl_join(_y, _x) __dl_join(_y, _x)
#define dl_join(_chan, _fun) _dl_join(_chan, _fun)



// Transport
#if DOWNLINK_TRANSPORT == PPRZ
   #define DefaultChannel  &transport_tx_PPRZ
#elif DOWNLINK_TRANSPORT == XBEE
   #define DefaultChannel  &transport_tx_XBEE
#else
   #error DOWNLINK defined but no DOWNLINK_TRANSPORT found.
#endif

// Device
// FIXME are DOWNLINK_AP|FBW_DEVICE distinction really necessary ?
// by default use AP_DEVICE if nothing is set ?
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#ifndef DefaultDevice
#define DefaultDevice dl_join(&dev_, DOWNLINK_DEVICE)
#endif


/** Counter of messages not sent because of unavailibity of the output buffer*/
extern uint8_t downlink_nb_ovrn;
extern uint16_t downlink_nb_bytes;
extern uint16_t downlink_nb_msgs;


#endif /* DOWNLINK_H */
