#if defined (USE_UART0) || defined (USE_UART1) || defined (USE_UART2) || defined (USE_UART3) || defined (USE_UART4) || defined (USE_UART5) || defined (USE_UART6) || defined (USE_SIM_UART)

#include "device_uart.h"
#include "mcu_periph/uart.h"
#include "simUart.h"

// DEV_UART FUNCTIONS -----------------------------------------------------------
//API functions declaration
// void     dev_uart_init(void* data);
// void     dev_uart_set_baudrate(void* data, uint32_t baudrate);
char*    dev_uart_name(void* data);
bool_t   dev_uart_register_transport(void* data, struct transport_rx* rx_tp);
struct transport_rx* dev_uart_rx_transport(void* data);
bool_t   dev_uart_check_free_space(void* data, uint8_t *slot_idx);
void     dev_uart_free_space(void* data, uint8_t slot_idx);
// uint8_t  dev_uart_transaction_length(void);
void     dev_uart_transaction_pack(void * trans, void * tx_data, uint16_t tx_length, void * rx_data, uint16_t rx_length, void (*callback)(void* trans));
void     dev_uart_sendMessage(void* data, uint8_t idx, void* trans, uint8_t priority);
uint8_t  dev_uart_getch(void* data);
bool_t   dev_uart_char_available(void* data);



//API functions definition
// void     dev_uart_init(void* data)                                                    { uart_periph_init((struct uart_periph*) data); }
// void     dev_uart_set_baudrate(void* data, uint32_t baudrate)                         { uart_periph_set_baudrate((struct uart_periph*) data, baudrate); }
char *   dev_uart_name(void* data)                                                    { return uart_periph_name((struct uart_periph*) data); }
bool_t   dev_uart_register_transport(void* data, struct transport_rx* rx_tp)          { return uart_register_transport((struct uart_periph*) data, rx_tp); }
struct transport_rx* dev_uart_rx_transport(void* data)                                { return uart_rx_transport((struct uart_periph*) data);}
bool_t   dev_uart_check_free_space(void* data, uint8_t *slot_idx)                     { return uart_check_free_space((struct uart_periph*) data, slot_idx); }
void     dev_uart_free_space(void* data, uint8_t slot_idx)                            { uart_free_space((struct uart_periph*) data, slot_idx); }
// uint8_t  dev_uart_transaction_length(void)                                              { return uart_transaction_length(); }
#include <string.h> //required for memcpy
void     dev_uart_transaction_pack(void * trans, void * tx_data, uint16_t tx_length, void * rx_data, uint16_t rx_length, void (*callback)(void* trans)) {
  //avoid warnings
  (void)rx_data;
  (void)rx_length;
// Due to align problems when working with dynamic buffer, transaction has to be filled in a local variable (aligned)
// and then moved to 'void' destiny trans in buffer (unaligned).
  struct uart_transaction tr;
  uart_transaction_pack(&tr, tx_data, tx_length, callback);
  memcpy(trans, &tr, sizeof(struct uart_transaction));
}
void     dev_uart_sendMessage(void* data, uint8_t idx, void* trans, uint8_t priority) { uart_sendMessage((struct uart_periph*) data, idx, trans, priority); }
uint8_t  dev_uart_getch(void* data)                                                   { return uart_getch((struct uart_periph*) data); }
bool_t   dev_uart_char_available(void* data)                                          { return uart_char_available((struct uart_periph*) data); }

#define INITIALIZED_DEV_UART_API { \
/*  .init               =    &dev_uart_init,*/ \
/*  .set_baudrate       =    &dev_uart_set_baudrate,*/ \
  .register_transport   =  &dev_uart_register_transport, \
  .rx_transport =          &dev_uart_rx_transport, \
  .name               =    &dev_uart_name, \
  .check_free_space   =    &dev_uart_check_free_space, \
  .free_space         =    &dev_uart_free_space, \
  .transaction_len    =    24, \
/*  .transaction_len    =    &dev_uart_transaction_length,*/ \
  .transaction_pack   =    &dev_uart_transaction_pack, \
  .transaction_summit =    &dev_uart_sendMessage, \
  .byte_available     =    &dev_uart_char_available, \
  .get_byte           =    &dev_uart_getch \
}



// DEV_UART STRUCTS -----------------------------------------------------------
#ifdef USE_UART0
struct device dev_UART0 = {
  .data   =    &uart0,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif

#ifdef USE_UART1
struct device dev_UART1 = {
  .data   =    &uart1,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_UART1

#ifdef USE_UART2
struct device dev_UART2 = {
  .data   =    &uart2,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_UART2

#ifdef USE_UART3
struct device dev_UART3 = {
  .data   =    &uart3,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_UART3

#ifdef USE_UART4
struct device dev_UART4 = {
  .data   =    &uart4,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_UART4

#ifdef USE_UART5
struct device dev_UART5 = {
  .data   =    &uart5,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_UART5

#ifdef USE_UART6
struct device dev_UART6 = {
  .data   =    &uart6,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_UART6

#ifdef USE_SIM_UART
struct device dev_SIM_UART = {
  .data   =    &sim_uart,
  .api    =    INITIALIZED_DEV_UART_API
};
#endif // USE_SIM_UART

#endif //defined (USE_UARTx) ||...

