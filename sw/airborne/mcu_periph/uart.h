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

/** \file mcu_periph/uart.h
 *  \brief arch independent UART (Universal Asynchronous Receiver/Transmitter) API
 *
 */

#ifndef MCU_PERIPH_UART_H
#define MCU_PERIPH_UART_H

#include "mcu_periph/uart_arch.h"
#include "std.h"
#include "transmit_queue.h"
#include "subsystems/datalink/transport2.h"

#define UART_RX_BUFFER_SIZE             128
#define UART_DEV_NAME_SIZE              16
#define UART_NUM_TRANSPORTS             2

/*
 * UART Baud rate defines in arch/x/mcu_periph/uart_arch.h
 */

#define UBITS_7 7
#define UBITS_8 8

#define USTOP_1 1
#define USTOP_2 2

#define UPARITY_NO    0
#define UPARITY_ODD   1
#define UPARITY_EVEN  2

/**
 * UART transaction
 */
struct uart_transaction {
  //transmition data
  void*     data;
  uint16_t  length;
  //callback
  void     (*callback)(void* this_transaction);
};
#define INITIALIZED_UART_TRANSACTION { \
  .length = 0, \
  .callback = NULL \
}

/**
 * UART peripheral
 */
struct uart_periph {
  /** Receive buffer */
  uint8_t rx_buf[UART_RX_BUFFER_SIZE];
  uint16_t rx_insert_idx;
  uint16_t rx_extract_idx;
  struct transport_rx* rx_tp;
  /** Transmit buffer */
  struct transmit_queue tx_queue;
  void* trans_p;                 // this should be 'struct uart_transaction*' but we have align problems with the dynamic_buffer.c
  struct uart_transaction trans; // due to align problems with dynamic_buffer.c, it is necessary a local copy of the transaction pointed by trans_p;
  uint8_t tx_byte_idx;
  uint8_t tx_running;
  /** UART Register */
  void* reg_addr;
  /** UART Dev (linux) */
  char dev[UART_DEV_NAME_SIZE];
  volatile uint16_t ore;    ///< overrun error counter
  volatile uint16_t ne_err; ///< noise error counter
  volatile uint16_t fe_err; ///< framing error counter
};
#define INITIALIZED_UART_PERIPH(_name) { \
  .rx_insert_idx = 0, \
  .rx_extract_idx = 0, \
  .rx_tp = NULL, \
  .tx_queue = INITIALIZED_TRANSMIT_QUEUE, \
  .trans_p = NULL, \
  .trans = INITIALIZED_UART_TRANSACTION, \
  .tx_byte_idx = 0, \
  .tx_running = FALSE, \
  .dev = _name, \
  .ore = 0, \
  .ne_err = 0, \
  .fe_err = 0 \
};

// -- GENERIC UART FUNCTIONS --------------------------------------------------
// -- 'UART Periph management' --
extern void uart_periph_init(struct uart_periph* p);
extern void uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud);
extern void uart_periph_set_bits_stop_parity(struct uart_periph* p, uint8_t bits, uint8_t stop, uint8_t parity);
extern void uart_periph_set_mode(struct uart_periph* p, bool_t tx_enabled, bool_t rx_enabled, bool_t hw_flow_control);
static inline char* uart_periph_name(struct uart_periph* p) {
  return (p->dev);
}

// -- 'UART data exchange' --
  //Tx queue
static inline bool_t uart_check_free_space(struct uart_periph* p, uint8_t *slot_idx) {
  return transmit_queue_check_free_space(&(p->tx_queue), slot_idx);
}
static inline void uart_free_space(struct uart_periph* p, uint8_t slot_idx) {
  transmit_queue_free_slot(&(p->tx_queue), slot_idx);
}
  //Tx transaction
static inline uint8_t uart_transaction_length(void) {
  return sizeof(struct uart_transaction);
}
static inline void uart_transaction_pack(struct uart_transaction* trans, void* data, uint16_t length, void (*callback)(void*)) {
  trans->data =     data;
  trans->length =   length;
  trans->callback = callback;
}
extern void uart_sendMessage(struct uart_periph *uart, uint8_t idx, void* trans, uint8_t priority);
  //Rx functions
static inline bool_t uart_register_transport(struct uart_periph* p, struct transport_rx* rx_tp) {
  if(p->rx_tp == NULL || p->rx_tp == rx_tp) {
    p->rx_tp = rx_tp;
    return TRUE;
  }
  return FALSE;
}
static inline struct transport_rx* uart_rx_transport(struct uart_periph* p) {
  return p->rx_tp;
}
static inline uint8_t uart_getch(struct uart_periph* p) {
  uint8_t ret = p->rx_buf[p->rx_extract_idx];
  p->rx_extract_idx = (p->rx_extract_idx + 1) % UART_RX_BUFFER_SIZE;
  return ret;
}
static inline bool_t uart_char_available(struct uart_periph* p) {
  return (p->rx_insert_idx != p->rx_extract_idx);
}
// -- end of 'UART data exchange' --




// -- SPECIFIC UART FUNCTIONS -------------------------------------------------

#ifdef USE_UART0
extern struct uart_periph uart0;

// -- 'UART Periph management' --
extern void uart0_init(void);
static inline void      UART0Init(void)                                         { uart_periph_init(&uart0); }
static inline uint8_t   UART0TxRunning(void)                                    { return uart0.tx_running; }
static inline void      UART0SetBaudrate(uint32_t baud)                         { uart_periph_set_baudrate(&uart0, baud); }
static inline void      UART0SetBitsStopParity(uint8_t bits, uint8_t stop, uint8_t parity)
                                                                                { uart_periph_set_bits_stop_parity(&uart0, bits, stop, parity); }
// -- 'UART data exchange' --
  //Tx queue
static inline bool_t    UART0CheckFreeSpace(uint8_t *slot_idx)                  { return uart_check_free_space(&uart0, slot_idx); }
static inline void      UART0FreeSpace(uint8_t slot_idx)                        { uart_free_space(&uart0, slot_idx); }
  //Tx transaction
static inline uint16_t  UART0TransactionLen(void)                               { return uart_transaction_length();}
static inline void      UART0TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                { uart_transaction_pack(trans, data, length, callback); }
static inline void      UART0SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)
                                                                                { uart_sendMessage(&uart0, slot_idx, trans, priority); }
  //Rx get char
static inline uint8_t   UART0Getch(void)                                        { return uart_getch(&uart0); }
static inline bool_t    UART0ChAvailable(void)                                  { return uart_char_available(&uart0); }

#endif // USE_UART0


#ifdef USE_UART1
extern struct uart_periph uart1;

// -- 'UART Periph management' --
extern void uart1_init(void);
static inline void      UART1Init(void)                                         { uart_periph_init(&uart1);}
static inline uint8_t   UART1TxRunning(void)                                    { return uart1.tx_running; }
static inline void      UART1SetBaudrate(uint32_t baud)                         { uart_periph_set_baudrate(&uart1, baud); }
static inline void      UART1SetBitsStopParity(uint8_t bits, uint8_t stop, uint8_t parity)
                                                                                { uart_periph_set_bits_stop_parity(&uart1, bits, stop, parity); }
// -- 'UART data exchange' --
  //Tx queue
static inline bool_t    UART1CheckFreeSpace(uint8_t *slot_idx)                  { return uart_check_free_space(&uart1, slot_idx); }
static inline void      UART1FreeSpace(uint8_t slot_idx)                        { uart_free_space(&uart1, slot_idx); }
  //Tx transaction
static inline uint16_t  UART1TransactionLen(void)                               { return uart_transaction_length();}
static inline void      UART1TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                { uart_transaction_pack(trans, data, length, callback); }
static inline void      UART1SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)
                                                                                { uart_sendMessage(&uart1, slot_idx, trans, priority); }
  //Rx get char
static inline uint8_t   UART1Getch(void)                                        { return uart_getch(&uart1);}
static inline bool_t    UART1ChAvailable(void)                                  { return uart_char_available(&uart1); }

#endif // USE_UART1


#ifdef USE_UART2
extern struct uart_periph uart2;

// -- 'UART Periph management' --
extern void uart2_init(void);
static inline void      UART2Init(void)                                         { uart_periph_init(&uart2);}
static inline uint8_t   UART2TxRunning(void)                                    { return uart2.tx_running; }
static inline void      UART2SetBaudrate(uint32_t baud)                         { uart_periph_set_baudrate(&uart2, baud); }
static inline void      UART2SetBitsStopParity(uint8_t bits, uint8_t stop, uint8_t parity)
                                                                                { uart_periph_set_bits_stop_parity(&uart2, bits, stop, parity); }
// -- 'UART data exchange' --
  //Tx queue
static inline bool_t    UART2CheckFreeSpace(uint8_t *slot_idx)                  { return uart_check_free_space(&uart2, slot_idx); }
static inline void      UART2FreeSpace(uint8_t slot_idx)                        { uart_free_space(&uart2, slot_idx); }
  //Tx transaction
static inline uint16_t  UART2TransactionLen(void)                               { return uart_transaction_length_inline();}
static inline void      UART2TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                { uart_transaction_pack_inline(trans, data, length, callback); }
static inline void      UART2SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)
                                                                                { uart_sendMessage(&uart2, slot_idx, trans, priority); }
  //Rx get char
static inline uint8_t   UART2Getch(void)                                        { return uart_getch(&uart2);}
static inline bool_t    UART2ChAvailable(void)                                  { return uart_char_available(&uart2); }

#endif // USE_UART2


#ifdef USE_UART3
extern struct uart_periph uart3;

// -- 'UART Periph management' --
extern void uart3_init(void);
static inline void      UART3Init(void)                                         { uart_periph_init(&uart3);}
static inline uint8_t   UART3TxRunning(void)                                    { return uart3.tx_running; }
static inline void      UART3SetBaudrate(uint32_t baud)                         { uart_periph_set_baudrate(&uart3, baud); }
static inline void      UART3SetBitsStopParity(uint8_t bits, uint8_t stop, uint8_t parity)
                                                                                { uart_periph_set_bits_stop_parity(&uart3, bits, stop, parity); }
// -- 'UART data exchange' --
  //Tx queue
static inline bool_t    UART3CheckFreeSpace(uint8_t *slot_idx)                  { return uart_check_free_space(&uart3, slot_idx); }
static inline void      UART3FreeSpace(uint8_t slot_idx)                        { uart_free_space(&uart3, slot_idx); }
  //Tx transaction
static inline uint16_t  UART3TransactionLen(void)                               { return uart_transaction_length_inline();}
static inline void      UART3TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                { uart_transaction_pack_inline(trans, data, length, callback); }
static inline void      UART3SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)
                                                                                { uart_sendMessage(&uart3, slot_idx, trans, priority); }
  //Rx get char
static inline uint8_t   UART3Getch(void)                                        { return uart_getch(&uart3);}
static inline bool_t    UART3ChAvailable(void)                                  { return uart_char_available(&uart3); }

#endif // USE_UART3


#ifdef USE_UART4
extern struct uart_periph uart4;

// -- 'UART Periph management' --
extern void uart4_init(void);
static inline void      UART4Init(void)                                         { uart_periph_init(&uart4);}
static inline uint8_t   UART4TxRunning(void)                                    { return uart4.tx_running; }
static inline void      UART4SetBaudrate(uint32_t baud)                         { uart_periph_set_baudrate(&uart4, baud); }
static inline void      UART4SetBitsStopParity(uint8_t bits, uint8_t stop, uint8_t parity)
                                                                                { uart_periph_set_bits_stop_parity(&uart4 bits, stop, parity); }
// -- 'UART data exchange' --
  //Tx queue
static inline bool_t    UART4CheckFreeSpace(uint8_t *slot_idx)                  { return uart_check_free_space(&uart4, slot_idx); }
static inline void      UART4FreeSpace(uint8_t slot_idx)                        { uart_free_space(&uart4, slot_idx); }
  //Tx transaction
static inline uint16_t  UART4TransactionLen(void)                               { return uart_transaction_length_inline();}
static inline void      UART4TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                { uart_transaction_pack_inline(trans, data, length, callback); }
static inline void      UART4SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)
                                                                                { uart_sendMessage(&uart4, slot_idx, trans, priority); }
  //Rx get char
static inline uint8_t   UART4Getch(void)                                        { return uart_getch(&uart4);}
static inline bool_t    UART4ChAvailable(void)                                  { return uart_char_available(&uart4); }

#endif // USE_UART4


#ifdef USE_UART5
extern struct uart_periph uart5;

// -- 'UART Periph management' --
extern void uart5_init(void);
static inline void      UART5Init(void)                                         { uart_periph_init(&uart5);}
static inline uint8_t   UART5TxRunning(void)                                    { return uart5.tx_running; }
static inline void      UART5SetBaudrate(uint32_t baud)                         { uart_periph_set_baudrate(&uart5, baud); }
static inline void      UART5SetBitsStopParity(uint8_t bits, uint8_t stop, uint8_t parity)
                                                                                { uart_periph_set_bits_stop_parity(&uart5, bits, stop, parity); }
// -- 'UART data exchange' --
  //Tx queue
static inline bool_t    UART5CheckFreeSpace(uint8_t *slot_idx)                  { return uart_check_free_space(&uart5, slot_idx); }
static inline void      UART5FreeSpace(uint8_t slot_idx)                        { uart_free_space(&uart5, slot_idx); }
  //Tx transaction
static inline uint16_t  UART5TransactionLen(void)                               { return uart_transaction_length_inline();}
static inline void      UART5TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                { uart_transaction_pack_inline(trans, data, length, callback); }
static inline void      UART5SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)
                                                                                { uart_sendMessage(&uart5, slot_idx, trans, priority); }
  //Rx get char
static inline uint8_t   UART5Getch(void)                                        { return uart_getch(&uart5);}
static inline bool_t    UART5ChAvailable(void)                                  { return uart_char_available(&uart5); }

#endif // USE_UART5


#ifdef USE_UART6
extern struct uart_periph uart6;

// -- 'UART Periph management' --
extern void uart6_init(void);
static inline void      UART6Init(void)                                         { uart_periph_init(&uart6);}
static inline uint8_t   UART6TxRunning(void)                                    { return uart6.tx_running; }
static inline void      UART6SetBaudrate(uint32_t baud)                         { uart_periph_set_baudrate(&uart6, baud); }
static inline void      UART6SetBitsStopParity(uint8_t bits, uint8_t stop, uint8_t parity)
                                                                                { uart_periph_set_bits_stop_parity(&uart6, bits, stop, parity); }
// -- 'UART data exchange' --
  //Tx queue
static inline bool_t    UART6CheckFreeSpace(uint8_t *slot_idx)                  { return uart_check_free_space(&uart6, slot_idx); }
static inline void      UART6FreeSpace(uint8_t slot_idx)                        { uart_free_space(&uart6, slot_idx); }
  //Tx transaction
static inline uint16_t  UART6TransactionLen(void)                               { return uart_transaction_length_inline();}
static inline void      UART6TransactionPack(void *trans, void* data, uint16_t length, void (*callback)(void*))
                                                                                { uart_transaction_pack_inline(trans, data, length, callback); }
static inline void      UART6SendMessage(uint8_t slot_idx, void* trans, uint8_t priority)
                                                                                { uart_sendMessage(&uart6, slot_idx, trans, priority); }
  //Rx get char
static inline uint8_t   UART6Getch(void)                                        { return uart_getch(&uart6);}
static inline bool_t    UART6ChAvailable(void)                                  { return uart_char_available(&uart6); }

#endif // USE_UART6

#endif /* MCU_PERIPH_UART_H */
