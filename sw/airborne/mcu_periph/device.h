/** \file mcu_periph.h
 *  \brief Interface for basic peripheric control
 * 
 *  This file provides a general API for controlling any peripheric device
 *  such as UART, SPI, I2C, etc.
 *  Just basic functions can be accessed through this API. For further control
 *  please refer to the particular peripheric device, such as 'uart.h' or 'spi.h'
 * 
 */
#ifndef _DEVICE_DRIVER_H_
#define _DEVICE_DRIVER_H_


#include <stdint.h>
#include "std.h"

struct device{
  bool_t   (*checkFreeSpace)(uint8_t length, uint8_t *idx);
  uint8_t* (*get_buff_pointer)(uint8_t idx);
  void     (*packMessage)(uint8_t idx, uint8_t offset, uint8_t *origin, uint8_t length);
  void     (*sendMessage)(uint8_t idx, uint8_t priority);
};


#ifdef USE_SIM_UART
extern struct device dev_SIM_UART;
#endif /* USE_SIM_UART */

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

#ifdef USE_I2C0
#endif /* USE_I2C0 */

#ifdef USE_I2C1
extern struct device dev_I2C1;
#endif /* USE_I2C1 */

#ifdef USE_I2C2
#endif /* USE_I2C2 */

#ifdef USE_I2C3
#endif /* USE_I2C3 */


#endif//_DEVICE_DRIVER_H_