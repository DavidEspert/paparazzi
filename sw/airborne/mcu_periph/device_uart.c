#include "device.h"
#include "uart.h"
#include "transmit_buffer.h"

#include <stdio.h> //for printf. Remove after debugging!

#define TRACE    printf
//#define TRACE    //


// UART ------------------------------------------------------------------------
#ifdef USE_UART0
struct device dev_UART0 = {
  .checkFreeSpace =     &uart0_checkFreeSpace,
  .get_buff_pointer =   &uart0_get_buff_pointer,
  .packMessage =        &uart0_packMessage,
  .sendMessage =        &uart0_sendMessage
};
#endif

#ifdef USE_UART1
struct device dev_UART1 = {
  .checkFreeSpace =     &uart1_checkFreeSpace,
  .get_buff_pointer =   &uart1_get_buff_pointer,
  .packMessage =        &uart1_packMessage,
  .sendMessage =        &uart1_sendMessage
};
#endif // USE_UART1

#ifdef USE_UART2
struct device dev_UART2 = {
  .checkFreeSpace =     &uart2_checkFreeSpace,
  .get_buff_pointer =   &uart2_get_buff_pointer,
  .packMessage =        &uart2_packMessage,
  .sendMessage =        &uart2_sendMessage
};
#endif // USE_UART2

#ifdef USE_UART3
struct device dev_UART3 = {
  .checkFreeSpace =     &uart3_checkFreeSpace,
  .get_buff_pointer =   &uart3_get_buff_pointer,
  .packMessage =        &uart3_packMessage,
  .sendMessage =        &uart3_sendMessage
};
#endif // USE_UART3

#ifdef USE_UART4
struct device dev_UART4 = {
  .checkFreeSpace =     &uart4_checkFreeSpace,
  .get_buff_pointer =   &uart4_get_buff_pointer,
  .packMessage =        &uart4_packMessage,
  .sendMessage =        &uart4_sendMessage
};
#endif // USE_UART4

#ifdef USE_UART5
struct device dev_UART5 = {
  .checkFreeSpace =     &uart5_checkFreeSpace,
  .get_buff_pointer =   &uart5_get_buff_pointer,
  .packMessage =        &uart5_packMessage,
  .sendMessage =        &uart5_sendMessage
};
#endif // USE_UART5

#ifdef USE_UART6
struct device dev_UART6 = {
  .checkFreeSpace =     &uart6_checkFreeSpace,
  .get_buff_pointer =   &uart6_get_buff_pointer,
  .packMessage =        &uart6_packMessage,
  .sendMessage =        &uart6_sendMessage
};
#endif // USE_UART6
