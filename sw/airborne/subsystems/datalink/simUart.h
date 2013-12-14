/** \file simUart.h
 *  \brief Interface for basic peripheric control
 * 
 *  This file provides an API for a simulated UART management as a generic peripheral device.
 *  API provides access to these functionalities:
 *  - check_free_space: Checks if there is a free transaction slot in UART Tx queue.
 *  - sendMessage: Summits a transaction in the reserved slot.
 *  - transaction_len: Returns UART transaction size.
 *
 *  User can directly use the static inline functions or, alternatively, access
 *  them through the 'struct device' interface.
 */

#ifndef _DEVICE_SIM_UART_H_
#define _DEVICE_SIM_UART_H_

#ifdef USE_SIM_UART

#include "mcu_periph/uart.h"

//Baudrate definitions
#define B1200    1200
#define B2400    2400
#define B4800    4800
#define B9600    9600
#define B19200   19200
#define B38400   38400
#define B57600   57600
#define B100000  100000
#define B115200  115200
#define B230400  230400

extern struct uart_periph sim_uart;

#endif /* USE_SIM_UART */

#endif // _DEVICE_SIM_UART_H_

