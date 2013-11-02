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

void uart_periph_init(struct uart_periph* p) {
  p->rx_insert_idx = 0;
  p->rx_extract_idx = 0;
  p->tx_insert_idx = 0;
  p->tx_extract_idx = 0;
  p->tx_running = FALSE;
  p->ore = 0;
  p->ne_err = 0;
  p->fe_err = 0;
}

bool_t uart_check_free_space(struct uart_periph* p, uint8_t len) {
  int16_t space = p->tx_extract_idx - p->tx_insert_idx;
  if (space <= 0)
    space += UART_TX_BUFFER_SIZE;
  return (uint16_t)(space - 1) >= len;
}

uint8_t uart_getch(struct uart_periph* p) {
  uint8_t ret = p->rx_buf[p->rx_extract_idx];
  p->rx_extract_idx = (p->rx_extract_idx + 1) % UART_RX_BUFFER_SIZE;
  return ret;
}

static inline void uart_sendMessage(struct uart_periph* p, uint8_t *buff, uint8_t length) {
  for(uint8_t i= 0; i < length; i++) uart_transmit(p, buff[i]);
}



#ifdef USE_UART0
struct uart_periph uart0;

void uart0_transmit(uint8_t data)			{ uart_transmit(&uart0, data); }
void uart0_sendMessage(uint8_t *buff, uint8_t length)	{ uart_sendMessage(&uart0, buff, length); }
bool_t uart0_checkFreeSpace(uint8_t length)		{ uart_check_free_space(&uart0, length); }
#endif

#ifdef USE_UART1
struct uart_periph uart1;

void uart1_transmit(uint8_t data)			{ uart_transmit(&uart1, data);}
void uart1_sendMessage(uint8_t *buff, uint8_t length)	{ uart_sendMessage(&uart1, buff, length); }
bool_t uart1_checkFreeSpace(uint8_t length)		{ uart_check_free_space(&uart1, length); }
#endif

#ifdef USE_UART2
struct uart_periph uart2;

void uart2_transmit(uint8_t data)			{ uart_transmit(&uart2, data);}
void uart2_sendMessage(uint8_t *buff, uint8_t length)	{ uart_sendMessage(&uart2, buff, length); }
bool_t uart2_checkFreeSpace(uint8_t length)		{ uart_check_free_space(&uart2, length); }
#endif

#ifdef USE_UART3
struct uart_periph uart3;

void uart3_transmit(uint8_t data)			{ uart_transmit(&uart3, data);}
void uart3_sendMessage(uint8_t *buff, uint8_t length)	{ uart_sendMessage(&uart3, buff, length); }
bool_t uart3_checkFreeSpace(uint8_t length)		{ uart_check_free_space(&uart3, length); }
#endif

#ifdef USE_UART4
struct uart_periph uart4;

void uart4_transmit(uint8_t data)			{ uart_transmit(&uart4, data);}
void uart4_sendMessage(uint8_t *buff, uint8_t length)	{ uart_sendMessage(&uart4, buff, length); }
bool_t uart4_checkFreeSpace(uint8_t length)		{ uart_check_free_space(&uart4, length); }
#endif

#ifdef USE_UART5
struct uart_periph uart5;

void uart5_transmit(uint8_t data)			{ uart_transmit(&uart5, data);}
void uart5_sendMessage(uint8_t *buff, uint8_t length)	{ uart_sendMessage(&uart5, buff, length); }
bool_t uart5_checkFreeSpace(uint8_t length)		{ uart_check_free_space(&uart5, length); }
#endif

#ifdef USE_UART6
struct uart_periph uart6;

void uart6_transmit(uint8_t data)			{ uart_transmit(&uart6, data);}
void uart6_sendMessage(uint8_t *buff, uint8_t length)	{ uart_sendMessage(&uart6, buff, length); }
bool_t uart6_checkFreeSpace(uint8_t length)		{ uart_check_free_space(&uart6, length); }
#endif
