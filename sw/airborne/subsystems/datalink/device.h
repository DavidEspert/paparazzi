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
#include "transport2.h"

struct transport_rx;  //even if "transport2.h" is included, forward definition is required due to circular dependencies.

struct device_api{
//   void                  (*init)(void* data);
//   void                  (*set_baudrate)(void* data, uint32_t baudrate);
  char*                 (*name)(void* data);
  bool_t                (*register_transport)(void* data, struct transport_rx* rx_tp);  //register always through datalink.h!
  struct transport_rx*  (*rx_transport)(void* data);

//Tx
  bool_t                (*check_free_space)(void* data, uint8_t *slot_idx);
  void                  (*free_space)(void* data, uint8_t slot_idx);
  uint8_t               transaction_len;
//   uint8_t               (*transaction_len)(void);
  void                  (*transaction_pack)(void * trans, void * tx_data, uint16_t tx_length, void * rx_data, uint16_t rx_length, void (*callback)(void* trans));
  void                  (*transaction_summit)(void* data, uint8_t idx, void* trans, uint8_t priority);
//Rx
  bool_t                (*byte_available)(void* data);
  uint8_t               (*get_byte)(void* data);
};

struct device{
  void * data;
  struct device_api api;
};

#endif//_DEVICE_H_