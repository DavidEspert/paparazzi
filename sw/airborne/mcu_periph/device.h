/** \file device.h
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
#include "device_uart.h"
#include "device_simUart.h"

struct device{
  bool_t   (*check_free_space)(uint8_t *idx);
  void     (*free_space)(uint8_t idx);
  uint8_t  (*transaction_len)(void);
  void     (*transaction_pack)(void *trans, void* data, uint8_t length, void (*callback)(void* trans));
  void     (*sendMessage)(uint8_t idx, void* transaction, uint8_t priority);
};

#endif//_DEVICE_DRIVER_H_