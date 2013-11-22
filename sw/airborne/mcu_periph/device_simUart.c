#ifdef USE_SIM_UART


#include "device_simUart.h"


#define _SIM_UART_DEBUG_

#ifdef _SIM_UART_DEBUG_
#include <stdio.h>
#define SIM_UART_TRACE(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#else
#define SIM_UART_TRACE(...)
#endif

//SIMULATED UART PARAMETERS
struct transmit_queue sim_tx_queue = INITIALIZED_TRANSMIT_QUEUE;
void *tr;
struct uart_transaction trans;
uint8_t tx_running = 0;
uint8_t tx_byte_idx = 0;
void    fake_uart_sendMessage(struct transmit_queue *tx_queue, uint8_t idx, void* transaction, uint8_t priority);
void    fake_ISR(struct transmit_queue *tx_queue);
void    fake_uart_transaction_pack(struct uart_transaction *trans, void* data, uint8_t length, void (*callback)(void*));

//AUXILIAR TEST PARAMETERS
uint8_t counter = 0;


// dev_SIM_UART FUNCTIONS ------------------------------------------------------------------------

//'Public' functions declaration (accessible through 'struct device dev_SIM_UART')
bool_t  SIM_UART_check_free_space(uint8_t *idx);
void    SIM_UART_sendMessage(uint8_t idx, void* transaction, uint8_t priority);
uint8_t SIM_UART_transaction_len(void);
void    SIM_UART_transaction_pack(void *trans, void* data, uint8_t length, void (*callback)(void* trans));
void    SIM_UART_transaction_free(uint8_t idx);

//'Public' functions definition 
bool_t  SIM_UART_check_free_space(uint8_t *idx)                                         { return dev_SIM_UART_check_free_space(idx); }
void    SIM_UART_sendMessage(uint8_t idx, void* transaction, uint8_t priority)          { dev_SIM_UART_sendMessage(idx, transaction, priority); }
uint8_t SIM_UART_transaction_len(void)                                                  { return dev_SIM_UART_transaction_len(); }
void    SIM_UART_transaction_pack(void *trans, void* data, uint8_t length, void (*callback)(void* trans))
                                                                                        { dev_SIM_UART_transaction_pack(trans, data, length, callback);}
void    SIM_UART_transaction_free(uint8_t idx)                                          { dev_SIM_UART_transaction_free(idx); }

//NON-INLINE 'Public' functions are accessible through 'struct device dev_SIM_UART'
struct device dev_SIM_UART = {
  .check_free_space =      &SIM_UART_check_free_space,
  .free_space =            &SIM_UART_transaction_free,
  .transaction_len =       &SIM_UART_transaction_len,
  .transaction_pack =      &SIM_UART_transaction_pack,
  .sendMessage =           &SIM_UART_sendMessage
};


// fake UART FUNCTIONS ------------------------------------------------------------------------

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

void fake_uart_transaction_pack(struct uart_transaction *trans, void* data, uint8_t length, void (*callback)(void*)) {
  trans->data =     data;
  trans->length =   length;
  trans->callback = callback;
}




#endif /* USE_SIM_UART */

