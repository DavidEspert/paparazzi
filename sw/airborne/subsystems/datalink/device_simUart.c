#ifdef USE_SIM_UART


#include "device_simUart.h"
#include "device_uart.h"
#include "mcu_periph/uart.h"
#include <string.h> //required for memcpy


#define _SIM_UART_TRACES_

#ifdef _SIM_UART_TRACES_
#include <stdio.h>
#define SIM_UART_TRACE(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#else
#define SIM_UART_TRACE(...)
#endif

//SIMULATED UART --------------------------------------------------------------
struct uart_periph sim_uart = INITIALIZED_UART_PERIPH("SIM_UART");
//AUXILIAR TEST PARAMETERS
uint8_t counter = 0;

  // any uart_arch.c is compiled and thus there is any 'uart_sendMessage' definition but a declaration...
void uart_sendMessage(struct uart_periph *uart, uint8_t idx, void* trans, uint8_t priority) __attribute__((alias("fake_uart_sendMessage")));
  // specific simulated uart functions
void fake_uart_sendMessage(struct uart_periph *p, uint8_t idx, void* transaction, uint8_t priority);
void fake_uart_ISR(struct uart_periph *p);

static inline void device_simUart_send_byte(struct uart_periph *p) {
  uint8_t data;
  uint16_t temp;

  //save data in local 'data' variable instead of transmit register...
  data = *((uint8_t*)(p->trans.data + p->tx_byte_idx++));
  SIM_UART_TRACE("\tdevice_simUart: send_byte:  SENDING BYTE %u (%u)...\n", p->tx_byte_idx, data);
  //save data in buffer in order to make a loop...
  p->rx_buf[p->rx_insert_idx] = data;
  temp = (p->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;
  if(temp != p->rx_extract_idx)
    p->rx_insert_idx = temp; // update insert index
}

void fake_uart_sendMessage(struct uart_periph *p, uint8_t idx, void* transaction, uint8_t priority) {
  //Insert message in transmit queue
  SIM_UART_TRACE("\n\tdevice_simUart: fake_uart_sendMessage:  INSERTING SLOT %u IN TRANSMIT QUEUE...\n", idx);
  transmit_queue_insert_slot(&(p->tx_queue), idx, transaction, priority);

  /* TEST 1: Send nothing */
//   return;

  /* TEST 2: Send everything */

  /* TEST 3: Send 1msg every 4msgs */
//   if(++counter == 4)    counter = 0;
//   else                  return;


  // check if in process of sending data
  if (p->tx_running) {
    // Nothing to do
  } else {
    SIM_UART_TRACE("\n\tdevice_simUart: fake_uart_sendMessage:  STARTING TRANSMITION...\n");
    if( transmit_queue_extract_slot(&(p->tx_queue), &(p->trans_p)) ){
      memcpy(&(p->trans), (p->trans_p), sizeof(struct uart_transaction));
      SIM_UART_TRACE("\tdevice_simUart: fake_uart_sendMessage:  SENDING MESSAGE (message length = %u)\n", p->trans.length);
      // set running flag and get a message to send
      p->tx_running = 1;
      p->tx_byte_idx = 0;
      device_simUart_send_byte(p);
      
      fake_uart_ISR(p);
    }
  }
}

void fake_uart_ISR(struct uart_periph *p) {

  while(1) {
    // check if more data to send in actual message
    if(p->tx_byte_idx < p->trans.length) {
      device_simUart_send_byte(p);
    }
    else{
      //message ended. Callback
      p->trans.callback(p->trans_p);

      /* TEST 1: send 1 message */
//       SIM_UART_TRACE("\tdevice_simUart: fake_uart_ISR:  STOP SENDING\n");
//       tx_running = 0;       // clear running flag
//       return;

      /* TEST 2: send all messages */
      // check if there is a new message
      if( transmit_queue_extract_slot(&(p->tx_queue), &(p->trans_p)) ){
        memcpy(&(p->trans), (p->trans_p), sizeof(struct uart_transaction));
        p->tx_byte_idx = 0;
        device_simUart_send_byte(p);
      }
      else{
        SIM_UART_TRACE("\tdevice_simUart: fake_uart_ISR:  NO MORE DATA TO SEND\n");
        p->tx_running = 0;       // clear running flag
        return;
      } 
    }       
  }
}



// dev_SIM_UART ---------------------------------------------------------------

//API functions declaration
  // real device_uart functions
extern char*    dev_uart_name(void* periph);
extern bool_t   dev_uart_check_free_space(void* periph, uint8_t *slot_idx);
extern void     dev_uart_free_space(void* periph, uint8_t slot_idx);
// extern uint8_t  dev_uart_transaction_length(void);
extern void     dev_uart_transaction_pack(void * trans, void * tx_data, uint16_t tx_length, void * rx_data, uint16_t rx_length, void (*callback)(void* trans));
  // specific device_simUart functions
void     dev_simUart_sendMessage(void* periph, uint8_t idx, void* trans, uint8_t priority);
uint8_t  dev_simUart_getch(void* periph);
bool_t   dev_simUart_char_available(void* periph);


//API functions definition
  // specific device_simUart functions
void     dev_simUart_sendMessage(void* periph, uint8_t idx, void* trans, uint8_t priority) { fake_uart_sendMessage((struct uart_periph*) periph, idx, trans, priority); }
uint8_t  dev_simUart_getch(void* periph)                                                   { return uart_getch((struct uart_periph*) periph); }
bool_t   dev_simUart_char_available(void* periph)                                          { return uart_char_available((struct uart_periph*) periph); }


//API functions declaration
#define INITIALIZED_DEV_SIM_UART_API { \
  .name               =    &dev_uart_name, \
  .check_free_space   =    &dev_uart_check_free_space, \
  .free_space         =    &dev_uart_free_space, \
  .transaction_len    =    24, \
/*  .transaction_len    =    &dev_uart_transaction_length,*/ \
  .transaction_pack   =    &dev_uart_transaction_pack, \
  .transaction_summit =    &dev_simUart_sendMessage, \
  .getch              =    &dev_simUart_getch, \
  .char_available     =    &dev_simUart_char_available \
}


struct device dev_SIM_UART = {
  .periph =    &sim_uart,
  .api = INITIALIZED_DEV_SIM_UART_API
};

#endif /* USE_SIM_UART */

