/*
 * Copyright (C) 2010 The Paparazzi Team
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

#include "mcu_periph/uart.h"

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"
#endif


#ifdef USE_UART0
struct uart_periph uart0;

// -- 'UART data exchange' (Non-inline functions: when function address is required...) --
  //Tx queue
bool_t   uart0_CheckFreeSpace(uint8_t *slot_idx);
uint16_t uart0_TransactionLen(void);
  //Tx transaction
void     uart0_TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*));
void     uart0_FreeSpace(uint8_t slot_idx);
void     uart0_SendMessage(uint8_t slot_idx, void* trans, uint8_t priority);

struct device dl_UART0 = {
  .check_free_space =      &uart0_CheckFreeSpace,
  .free_space       =      &uart0_FreeSpace,
  .transaction_len =       &uart_transaction_length,
  .transaction_pack =      &uart_transaction_pack,
  .sendMessage =           &uart0_SendMessage
};

bool_t   uart0_CheckFreeSpace(uint8_t *slot_idx)                                 { return UART0CheckFreeSpace(slot_idx); }
uint16_t uart0_TransactionLen(void)                                              { return UART0TransactionLen(); }
void     uart0_TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                 { UART0TransactionPack(trans, data, length, callback); }
void     uart0_FreeSpace(uint8_t slot_idx)                                       { UART0FreeSpace(slot_idx); }
void     uart0_SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)      { UART0SendMessage(slot_idx, trans, priority); }

#if DOWNLINK
static void send_uart0_err(void) {
  uint16_t ore    = uart0.ore;
  uint16_t ne_err = uart0.ne_err;
  uint16_t fe_err = uart0.fe_err;
  const uint8_t _bus0 = 0;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus0);
}
#endif

#endif

#ifdef USE_UART1
struct uart_periph uart1;

// -- 'UART data exchange' (Non-inline functions: when function address is required...) --
  //Tx queue
bool_t   uart1_CheckFreeSpace(uint8_t *slot_idx);
void     uart1_FreeSpace(uint8_t slot_idx);
  //Tx transaction
uint16_t uart1_TransactionLen(void);
void     uart1_TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*));
void     uart1_SendMessage(uint8_t slot_idx, void* trans, uint8_t priority);

struct device dl_UART1 = {
  .check_free_space =      &uart1_CheckFreeSpace,
  .free_space       =      &uart1_FreeSpace,
  .transaction_len =       &uart_transaction_length,
  .transaction_pack =      &uart_transaction_pack,
  .sendMessage =           &uart1_SendMessage
};

bool_t   uart1_CheckFreeSpace(uint8_t *slot_idx)                                 { return UART1CheckFreeSpace(slot_idx); }
uint16_t uart1_TransactionLen(void)                                              { return UART1TransactionLen(); }
void     uart1_TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                 { UART1TransactionPack(trans, data, length, callback); }
void     uart1_FreeSpace(uint8_t slot_idx)                                       { UART1FreeSpace(slot_idx); }
void     uart1_SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)      { UART1SendMessage(slot_idx, trans, priority); }

#if DOWNLINK
static void send_uart1_err(void) {
  uint16_t ore    = uart1.ore;
  uint16_t ne_err = uart1.ne_err;
  uint16_t fe_err = uart1.fe_err;
  const uint8_t _bus1 = 1;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus1);
}
#endif

#endif

#ifdef USE_UART2
struct uart_periph uart2;

bool_t   uart2_CheckFreeSpace(uint8_t *slot_idx)                                 { return UART2CheckFreeSpace(slot_idx); }
uint16_t uart2_TransactionLen(void)                                              { return UART2TransactionLen(); }
void     uart2_TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                 { UART2TransactionPack(trans, data, length, callback); }
void     uart2_FreeSpace(uint8_t slot_idx)                                       { UART2FreeSpace(slot_idx); }
void     uart2_SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)      { UART2SendMessage(slot_idx, trans, priority); }

#if DOWNLINK
static void send_uart2_err(void) {
  uint16_t ore    = uart2.ore;
  uint16_t ne_err = uart2.ne_err;
  uint16_t fe_err = uart2.fe_err;
  const uint8_t _bus2 = 2;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus2);
}
#endif

#endif

#ifdef USE_UART3
struct uart_periph uart3;

bool_t   uart3_CheckFreeSpace(uint8_t *slot_idx)                                 { return UART3CheckFreeSpace(slot_idx); }
uint16_t uart3_TransactionLen(void)                                              { return UART3TransactionLen(); }
void     uart3_TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                 { UART3TransactionPack(trans, data, length, callback); }
void     uart3_FreeSpace(uint8_t slot_idx)                                       { UART3FreeSpace(slot_idx); }
void     uart3_SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)      { UART3SendMessage(slot_idx, trans, priority); }

#if DOWNLINK
static void send_uart3_err(void) {
  uint16_t ore    = uart3.ore;
  uint16_t ne_err = uart3.ne_err;
  uint16_t fe_err = uart3.fe_err;
  const uint8_t _bus3 = 3;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus3);
}
#endif

#endif

#ifdef USE_UART4
struct uart_periph uart4;

bool_t   uart4_CheckFreeSpace(uint8_t *slot_idx)                                 { return UART4CheckFreeSpace(slot_idx); }
uint16_t uart4_TransactionLen(void)                                              { return UART4TransactionLen(); }
void     uart4_TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                 { UART4TransactionPack(trans, data, length, callback); }
void     uart4_FreeSpace(uint8_t slot_idx)                                       { UART4FreeSpace(slot_idx); }
void     uart4_SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)      { UART4SendMessage(slot_idx, trans, priority); }

#if DOWNLINK
static void send_uart4_err(void) {
  uint16_t ore    = uart4.ore;
  uint16_t ne_err = uart4.ne_err;
  uint16_t fe_err = uart4.fe_err;
  const uint8_t _bus4 = 4;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus4);
}
#endif

#endif

#ifdef USE_UART5
struct uart_periph uart5;

bool_t   uart5_CheckFreeSpace(uint8_t *slot_idx)                                 { return UART5CheckFreeSpace(slot_idx); }
uint16_t uart5_TransactionLen(void)                                              { return UART5TransactionLen(); }
void     uart5_TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                 { UART5TransactionPack(trans, data, length, callback); }
void     uart5_FreeSpace(uint8_t slot_idx)                                       { UART5FreeSpace(slot_idx); }
void     uart5_SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)      { UART5SendMessage(slot_idx, trans, priority); }

#if DOWNLINK
static void send_uart5_err(void) {
  uint16_t ore    = uart5.ore;
  uint16_t ne_err = uart5.ne_err;
  uint16_t fe_err = uart5.fe_err;
  const uint8_t _bus5 = 5;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus5);
}
#endif

#endif

#ifdef USE_UART6
struct uart_periph uart6;

bool_t   uart6_CheckFreeSpace(uint8_t *slot_idx)                                 { return UART6CheckFreeSpace(slot_idx); }
uint16_t uart6_TransactionLen(void)                                              { return UART6TransactionLen(); }
void     uart6_TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                 { UART6TransactionPack(trans, data, length, callback); }
void     uart6_FreeSpace(uint8_t slot_idx)                                       { UART6FreeSpace(slot_idx); }
void     uart6_SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)      { UART6SendMessage(slot_idx, trans, priority); }

#if DOWNLINK
static void send_uart6_err(void) {
  const uint8_t _bus6 = 6;
  uint16_t ore    = uart6.ore;
  uint16_t ne_err = uart6.ne_err;
  uint16_t fe_err = uart6.fe_err;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus6);
}
#endif

#endif

#if DOWNLINK
static void send_uart_err(void) {
  static uint8_t uart_nb_cnt = 0;
  switch (uart_nb_cnt) {
#if USE_UART0
    case 0:
      send_uart0_err(); break;
#endif
#if USE_UART1
    case 1:
      send_uart1_err(); break;
#endif
#if USE_UART2
    case 2:
      send_uart2_err(); break;
#endif
#if USE_UART3
    case 3:
      send_uart3_err(); break;
#endif
#if USE_UART4
    case 4:
      send_uart4_err(); break;
#endif
#if USE_UART5
    case 5:
      send_uart5_err(); break;
#endif
#if USE_UART6
    case 6:
      send_uart6_err(); break;
#endif
    default: break;
  }
  uart_nb_cnt++;
  if (uart_nb_cnt == 6)
    uart_nb_cnt = 0;
}
#endif

//uart_periph_init: Initialize uart_periph struct
void uart_periph_init(struct uart_periph* p) {
  p->rx_insert_idx = 0;
  p->rx_extract_idx = 0;
  transmit_queue_init(&(p->tx_queue));
  p->tx_byte_idx = 0;
  p->tx_running = FALSE;
  p->ore = 0;
  p->ne_err = 0;
  p->fe_err = 0;

#if DOWNLINK
  // the first to register do it for the others
  register_periodic_telemetry(DefaultPeriodic, "UART_ERRORS", send_uart_err);
#endif
}

uint16_t uart_transaction_length(void) {
  return uart_transaction_length_inline();
}

void uart_transaction_pack(void *trans, void* data, uint16_t length, void (*callback)(void*)) {
  uart_transaction_pack_inline(trans, data, length, callback);
}

