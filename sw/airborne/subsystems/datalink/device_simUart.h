/** \file device_simUart.h
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

#include "device.h"
#include "mcu_periph/uart.h"
#include <string.h> //required for memcpy

extern struct transmit_queue sim_tx_queue;
extern void fake_uart_sendMessage(struct transmit_queue *tx_queue, uint8_t idx, void* transaction, uint8_t priority);
extern void fake_uart_transaction_pack(struct uart_transaction *trans, void* data, uint16_t length, void (*callback)(void*));

//INLINE 'Public' functions
static inline bool_t   dev_SIM_UART_check_free_space(uint8_t *idx)      { return transmit_queue_check_free_space(&sim_tx_queue, idx); }
static inline void     dev_SIM_UART_sendMessage(uint8_t idx, void* transaction, uint8_t priority)
                                                                        { fake_uart_sendMessage(&sim_tx_queue, idx, transaction, priority); }
static inline uint16_t dev_SIM_UART_transaction_len(void)               { return sizeof(struct uart_transaction); }
static inline void     dev_SIM_UART_transaction_pack(void *trans, void* data, uint16_t length, void (*callback)(void* trans)) {
// Due to align problems in dynamic buffer, transaction has to be filled in a local varialble (aligned)
// and then moved to its destiny in buffer (unaligned).
// If you're sure that trans is correctly aligned just execute
//  fake_uart_transaction_pack((struct uart_transaction*) trans, data, length, callback);
  struct uart_transaction tr;
  
  fake_uart_transaction_pack(&tr, data, length, callback);
  memcpy(trans, &tr, sizeof(struct uart_transaction));
}
static inline void    dev_SIM_UART_transaction_free(uint8_t idx)        { transmit_queue_free_slot(&sim_tx_queue, idx); }



//'Public' functions are also accessible through 'struct device dev_SIM_UART'
extern struct device dev_SIM_UART;

#endif /* USE_SIM_UART */

#endif // _DEVICE_SIM_UART_H_

