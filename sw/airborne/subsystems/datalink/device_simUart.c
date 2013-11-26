#ifdef USE_SIM_UART


#include "device_simUart.h"
#include "mcu_periph/uart.h"
#include <string.h> //required for memcpy


#define _SIM_UART_DEBUG_

#ifdef _SIM_UART_DEBUG_
#include <stdio.h>
#define SIM_UART_TRACE(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#else
#define SIM_UART_TRACE(...)
#endif

//SIMULATED UART --------------------------------------------------------------
struct transmit_queue sim_tx_queue = INITIALIZED_TRANSMIT_QUEUE;
void *tr;
struct uart_transaction trans;
uint8_t tx_running = 0;
uint8_t tx_byte_idx = 0;
void    fake_uart_sendMessage(struct transmit_queue *tx_queue, uint8_t idx, void* transaction, uint8_t priority);
void    fake_ISR(struct transmit_queue *tx_queue);
void    fake_uart_transaction_pack(struct uart_transaction *trans, void* data, uint16_t length, void (*callback)(void*));

//AUXILIAR TEST PARAMETERS
uint8_t counter = 0;

void fake_uart_sendMessage(struct transmit_queue *tx_queue, uint8_t idx, void* transaction, uint8_t priority) {
  uint8_t data;

  //Insert message in transmit queue
  SIM_UART_TRACE("\n\tdevice_simUart: dev_SIM_sendMessage:  INSERTING SLOT %u IN TRANSMIT QUEUE...\n", idx);
  transmit_queue_insert_slot(tx_queue, idx, transaction, priority);

  /* TEST 1: Send nothing */
//   return;

  /* TEST 2: Send everything */

  /* TEST 3: Send 1msg every 4msgs */
//   if(++counter == 4)    counter = 0;
//   else                  return;


  // check if in process of sending data
  if (tx_running) {
    // Nothing to do
  } else {
    SIM_UART_TRACE("\n\tdevice_simUart: dev_SIM_sendMessage:  STARTING TRANSMITION...\n");
    if( transmit_queue_extract_slot(tx_queue, &tr) ){
      memcpy(&trans, tr, sizeof(struct uart_transaction));
      SIM_UART_TRACE("\tdevice_simUart: dev_SIM_sendMessage:  SENDING MESSAGE (message length = %u)\n", trans.length);
      // set running flag and get a message to send
      tx_running = 1;
      tx_byte_idx = 0;
      data = *((uint8_t*)(trans.data + tx_byte_idx++));
      SIM_UART_TRACE("\tdevice_simUart: dev_SIM_sendMessage:  SENDING BYTE %u (%u)...\n", tx_byte_idx, data);
      
      fake_ISR(tx_queue);
    }
  }
}

void fake_ISR(struct transmit_queue *tx_queue) {
  uint8_t data;

  while(1) {
    // check if more data to send in actual message
    if(tx_byte_idx < trans.length) {
      data = *((uint8_t*)(trans.data + tx_byte_idx++));
      SIM_UART_TRACE("\tdevice_simUart: fake_ISR:  SENDING BYTE %u (%u)...\n", tx_byte_idx, data);
    }
    else{
      //message ended. Callback
      trans.callback(tr);

      /* TEST 1: send 1 message */
//       SIM_UART_TRACE("\tdevice_simUart: fake_ISR:  STOP SENDING\n");
//       tx_running = 0;       // clear running flag
//       return;

      /* TEST 2: send all messages */
      // check if there is a new message
      if( transmit_queue_extract_slot(tx_queue, &tr) ){
       memcpy(&trans, tr, sizeof(struct uart_transaction));
       tx_byte_idx = 0;
        data = *((uint8_t*)(trans.data + tx_byte_idx++));
        SIM_UART_TRACE("\tdevice_simUart: fake_ISR:  SENDING BYTE %u (%u)...\n", tx_byte_idx, data);
      }
      else{
        SIM_UART_TRACE("\tdevice_simUart: fake_ISR:  NO MORE DATA TO SEND\n");
        tx_running = 0;       // clear running flag
        return;
      } 
    }       
  }
}

void fake_uart_transaction_pack(struct uart_transaction *trans, void* data, uint16_t length, void (*callback)(void*)) {
  trans->data =     data;
  trans->length =   length;
  trans->callback = callback;
}



// dev_SIM_UART ---------------------------------------------------------------

//API functions declaration
bool_t   dev_simUart_check_free_space(void* periph, uint8_t *slot_idx);
void     dev_simUart_free_space(void* periph, uint8_t slot_idx);
// uint8_t  dev_simUart_transaction_length(void);
void     dev_simUart_transaction_pack(void *trans, void* data, uint16_t length, void (*callback)(void*));
void     dev_simUart_sendMessage(void* periph, uint8_t idx, void* trans, uint8_t priority);
uint8_t  dev_simUart_getch(void* periph);
bool_t   dev_simUart_char_available(void* periph);



//API functions definition
bool_t   dev_simUart_check_free_space(void* periph, uint8_t *slot_idx)                     { return transmit_queue_check_free_space(&sim_tx_queue, slot_idx); }
void     dev_simUart_free_space(void* periph, uint8_t slot_idx)                            { transmit_queue_free_slot(&sim_tx_queue, slot_idx); }
// uint8_t  dev_simUart_transaction_length(void)                                              { return uart_transaction_length(); }
void     dev_simUart_transaction_pack(void *trans, void* data, uint16_t length, void (*callback)(void*)) {
// Due to align problems when working with dynamic buffer, transaction has to be filled in a local variable (aligned)
// and then moved to 'void' destiny trans in buffer (unaligned).
  struct uart_transaction tr;
  fake_uart_transaction_pack(&tr, data, length, callback);
  memcpy(trans, &tr, sizeof(struct uart_transaction));
}
void     dev_simUart_sendMessage(void* periph, uint8_t idx, void* trans, uint8_t priority) { fake_uart_sendMessage(&sim_tx_queue, idx, trans, priority); }
uint8_t  dev_simUart_getch(void* periph)                                                   { return 0; }// return uart_getch((struct uart_periph*) periph); }
bool_t   dev_simUart_char_available(void* periph)                                          { return FALSE; }// return uart_char_available((struct uart_periph*) periph); }

#define INITIALIZED_DEV_UART_API { \
  .check_free_space   =    &dev_simUart_check_free_space, \
  .free_space         =    &dev_simUart_free_space, \
  .transaction_len    =    24, \
/*  .transaction_len    =    &dev_simUart_transaction_length,*/ \
  .transaction_pack   =    &dev_simUart_transaction_pack, \
  .transaction_summit =    &dev_simUart_sendMessage, \
  .getch              =    &dev_simUart_getch, \
  .char_available     =    &dev_simUart_char_available \
}



struct device dev_SIM_UART = {
  .api = INITIALIZED_DEV_UART_API
};

#endif /* USE_SIM_UART */

