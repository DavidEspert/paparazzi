/** \file device_uart.h
 *  \brief Interface for basic peripheric control
 * 
 *  This file provides an API for UART management as a generic peripheral device.
 *  API provides access to these functionalities:
 *  - check_free_space: Checks if there is a free transaction slot in UART Tx queue.
 *  - sendMessage:      Summits a transaction in the reserved slot.
 *  - transaction_len:  Returns UART transaction size.
 *
 *  User can directly use the static inline functions or, alternatively, access
 *  them through the 'struct device' interface.
 */

#ifndef _DEVICE_UART_H_
#define _DEVICE_UART_H_

#include "device.h"

#ifdef USE_UART0
extern struct device dev_UART0;
#endif // USE_UART0

#ifdef USE_UART1
extern struct device dev_UART1;
#endif // USE_UART1

#ifdef USE_UART2
extern struct device dev_UART2;
#endif // USE_UART2

#ifdef USE_UART3
extern struct device dev_UART3;
#endif // USE_UART3

#ifdef USE_UART4
extern struct device dev_UART4;
#endif // USE_UART4

#ifdef USE_UART5
extern struct device dev_UART5;
#endif // USE_UART5

#ifdef USE_UART6
extern struct device dev_UART6;
#endif // USE_UART6

#ifdef USE_SIM_UART
extern struct device dev_SIM_UART;
#endif // USE_SIM_UART

#endif // _DEVICE_UART_H_

