#include "device_uart.h"

void dev_uart_transaction_pack(void* trans, void* data, uint8_t length, void (*callback)(void*));
void dev_uart_transaction_pack(void* trans, void* data, uint8_t length, void (*callback)(void*))
{ dev_uart_transaction_pack_inline(trans, data, length, callback); }


// UART ------------------------------------------------------------------------
#ifdef USE_UART0
struct device dev_UART0 = {
  .check_free_space =      &uart0_CheckFreeSpace,
  .free_space       =      &uart0_FreeSpace,
  .transaction_len =       &uart_transaction_length,
  .transaction_pack =      &dev_uart_transaction_pack,
  .sendMessage =           &uart0_SendMessage
};
#endif

#ifdef USE_UART1
struct device dev_UART1 = {
  .check_free_space =      &uart1_CheckFreeSpace,
  .free_space       =      &uart1_FreeSpace,
  .transaction_len =       &uart_transaction_length,
  .transaction_pack =      &dev_uart_transaction_pack,
  .sendMessage =           &uart1_SendMessage
};
#endif // USE_UART1

#ifdef USE_UART2
struct device dev_UART2 = {
  .check_free_space =      &uart2_CheckFreeSpace,
  .free_space       =      &uart2_FreeSpace,
  .transaction_len =       &uart_transaction_length,
  .transaction_pack =      &dev_uart_transaction_pack,
  .sendMessage =           &uart2_SendMessage
};
#endif // USE_UART2

#ifdef USE_UART3
struct device dev_UART3 = {
  .check_free_space =      &uart3_CheckFreeSpace,
  .free_space       =      &uart3_FreeSpace,
  .transaction_len =       &uart_transaction_length,
  .transaction_pack =      &dev_uart_transaction_pack,
  .sendMessage =           &uart3_SendMessage
};
#endif // USE_UART3

#ifdef USE_UART4
struct device dev_UART4 = {
  .check_free_space =      &uart4_CheckFreeSpace,
  .free_space       =      &uart4_FreeSpace,
  .transaction_len =       &uart_transaction_length,
  .transaction_pack =      &dev_uart_transaction_pack,
  .sendMessage =           &uart4_SendMessage
};
#endif // USE_UART4

#ifdef USE_UART5
struct device dev_UART5 = {
  .check_free_space =      &uart5_CheckFreeSpace,
  .free_space       =      &uart5_FreeSpace,
  .transaction_len =       &uart_transaction_length,
  .transaction_pack =      &dev_uart_transaction_pack,
  .sendMessage =           &uart5_SendMessage
};
#endif // USE_UART5

#ifdef USE_UART6
struct device dev_UART6 = {
  .check_free_space =      &uart6_CheckFreeSpace,
  .free_space       =      &uart6_FreeSpace,
  .transaction_len =       &uart_transaction_length,
  .transaction_pack =      &dev_uart_transaction_pack,
  .sendMessage =           &uart6_SendMessage
};


#endif // USE_UART6
