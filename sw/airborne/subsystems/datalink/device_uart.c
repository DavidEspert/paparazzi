#include "device_uart.h"
#include "mcu_periph/uart.h"

//API functions declaration
bool_t   dev_uart_check_free_space(void* periph, uint8_t *slot_idx);
void     dev_uart_free_space(void* periph, uint8_t slot_idx);
// uint8_t  dev_uart_transaction_length(void);
void     dev_uart_transaction_pack(void *trans, void* data, uint16_t length, void (*callback)(void*));
void     dev_uart_sendMessage(void* periph, uint8_t idx, void* trans, uint8_t priority);
// bool_t   dev_uart_add_rx_transport(void* periph, struct transport2* rx_tp);
uint8_t  dev_uart_getch(void* periph);
bool_t   dev_uart_char_available(void* periph);

//API functions definition
bool_t   dev_uart_check_free_space(void* periph, uint8_t *slot_idx)                     { return uart_check_free_space((struct uart_periph*) periph, slot_idx); }
void     dev_uart_free_space(void* periph, uint8_t slot_idx)                            { uart_free_space((struct uart_periph*) periph, slot_idx); }
// uint8_t  dev_uart_transaction_length(void)                                              { return uart_transaction_length(); }
#include <string.h> //required for memcpy
void     dev_uart_transaction_pack(void *trans, void* data, uint16_t length, void (*callback)(void*)) {
// Due to align problems when working with dynamic buffer, transaction has to be filled in a local variable (aligned)
// and then moved to 'void' destiny trans in buffer (unaligned).
  struct uart_transaction tr;
  uart_transaction_pack(&tr, data, length, callback);
  memcpy(trans, &tr, sizeof(struct uart_transaction));
}
void     dev_uart_sendMessage(void* periph, uint8_t idx, void* trans, uint8_t priority) { uart_sendMessage((struct uart_periph*) periph, idx, trans, priority); }
// bool_t   dev_uart_add_rx_transport(void* periph, struct transport2* rx_tp)              { return uart_add_rx_transport((struct uart_periph*) periph, rx_tp); }
uint8_t  dev_uart_getch(void* periph)                                                   { return uart_getch((struct uart_periph*) periph); }
bool_t   dev_uart_char_available(void* periph)                                          { return uart_char_available((struct uart_periph*) periph); }

#define INITIALIZED_DEV_UART_API { \
  .check_free_space   =    &dev_uart_check_free_space, \
  .free_space         =    &dev_uart_free_space, \
  .transaction_len    =    24, \
/*  .transaction_len    =    &dev_uart_transaction_length,*/ \
  .transaction_pack   =    &dev_uart_transaction_pack, \
  .transaction_summit =    &dev_uart_sendMessage, \
/*  .add_rx_transport   =    &dev_uart_add_rx_transport,*/ \
  .getch              =    &dev_uart_getch, \
  .char_available     =    &dev_uart_char_available \
}

// DEV_UART ------------------------------------------------------------------------
#ifdef USE_UART0
struct device dev_UART0 = {
  .periph =    &uart0,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif

#ifdef USE_UART1
struct device dev_UART1 = {
  .periph =    &uart1,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_UART1

#ifdef USE_UART2
struct device dev_UART2 = {
  .periph =    &uart2,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_UART2

#ifdef USE_UART3
struct device dev_UART3 = {
  .periph =    &uart3,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_UART3

#ifdef USE_UART4
struct device dev_UART4 = {
  .periph =    &uart4,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_UART4

#ifdef USE_UART5
struct device dev_UART5 = {
  .periph =    &uart5,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_UART5

#ifdef USE_UART6
struct device dev_UART6 = {
  .periph =    &uart6,
  .api    =    INITIALIZED_DEV_UART_API
};


#endif // USE_UART6
