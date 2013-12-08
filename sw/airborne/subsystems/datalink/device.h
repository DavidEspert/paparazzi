/** \file device.h
 *  \brief Interface for basic peripheric control
 * 
 *  This file provides a general API for controlling any peripheric device
 *  such as UART, SPI, I2C, etc.
 *  Just basic functions can be accessed through this API. For further control
 *  please refer to the particular peripheric device, such as 'uart.h' or 'spi.h'
 * 
 */
#ifndef _DEVICE_H_
#define _DEVICE_H_


#include <stdint.h>
#include "std.h"

struct device_api{
  char*    (*name)(void* periph);
//Tx
  bool_t   (*check_free_space)(void* periph, uint8_t *slot_idx);
  void     (*free_space)(void* periph, uint8_t slot_idx);
  uint8_t  transaction_len;
//   uint8_t  (*transaction_len)(void);
  void     (*transaction_pack)(void * trans, void * tx_data, uint16_t tx_length, void * rx_data, uint16_t rx_length, void (*callback)(void* trans));
  void     (*transaction_summit)(void* periph, uint8_t idx, void* trans, uint8_t priority);
//Rx
//   bool_t   (*add_rx_transport)(void* periph, struct transport2* rx_tp);
  bool_t   (*char_available)(void* periph);
  uint8_t  (*getch)(void* periph);
};

struct device{
  void * periph;
  struct device_api api;
};

#endif//_DEVICE_H_