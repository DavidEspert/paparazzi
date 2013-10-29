#include "device.h"
#include "uart.h"
#include "transmit_buffer.h"

#include <stdio.h> //for printf. Remove after debugging!

#define TRACE    printf
//#define TRACE    //


// UART ------------------------------------------------------------------------
void dev_UART_sendMessage(struct uart_periph *uart, struct transmit_buffer *tx_buff, uint8_t idx, uint8_t priority) {
  uint8_t temp;
  //Insert message in transmit queue
  try_insert_slot_in_queue(tx_buff, idx, priority);

  //Insert as messages as possible in uart's Fifo.
  while(tx_buff->first_output < TX_BUFF_NUM_SLOTS){
    tx_buff->slot[tx_buff->first_output].status = ST_SENDING; //freeze this message as first output
    if(uart_check_free_space(uart, tx_buff->slot[tx_buff->first_output].length)){
      for(uint8_t i= 0; i < tx_buff->slot[tx_buff->first_output].length; i++)
	uart_transmit(uart, tx_buff->slot[tx_buff->first_output].buffer[i]);
      //message is sent. Free this slot
      temp = tx_buff->first_output;
      tx_buff->first_output = tx_buff->slot[tx_buff->first_output].next_slot;
      tx_buffer_free_slot(tx_buff, temp);
      TRACE("\tdevice: dev_UART_sendMessage:  message from slot %u passed to UART\n", tx_buff->first_output);
    }
    else{
      tx_buff->slot[tx_buff->first_output].status = ST_READY; //Unfreeze this message as first output
      break;
    }
  }
}



#ifdef USE_UART0
struct transmit_buffer uart0_tx_buff = INITIALIZED_TX_BUFFER;

uint8_t dev_UART0_get_tx_slot(uint8_t length, uint8_t *idx)			{ return tx_buffer_get_slot(&uart0_tx_buff, length, idx); }
uint8_t * dev_UART0_get_buff_pointer(uint8_t idx)				{ return tx_buffer_get_slot_pointer(&uart0_tx_buff, idx); }
void dev_UART0_sendMessage(uint8_t idx, uint8_t priority)			{ dev_UART_sendMessage(&uart0, &uart0_tx_buff, idx, priority); }

struct device dev_UART0 = {
//  .init = &uart0_init,
//  .checkFreeSpace =	&uart0_checkFreeSpace,
//  .transmit =		&uart0_transmit,
  .get_tx_slot =	&dev_UART0_get_tx_slot,
  .get_buff_pointer =	&dev_UART0_get_buff_pointer,
  .sendMessage =	&dev_UART0_sendMessage
};
#endif

#ifdef USE_UART1
struct device dev_UART1;
#endif // USE_UART1

#ifdef USE_UART2
struct device dev_UART2;
#endif // USE_UART2

#ifdef USE_UART3
struct device dev_UART3;
#endif // USE_UART3

#ifdef USE_UART4
struct device dev_UART4;
#endif // USE_UART4

#ifdef USE_UART5
struct device dev_UART5;
#endif // USE_UART5

#ifdef USE_UART6
struct device dev_UART6;
#endif // USE_UART6
