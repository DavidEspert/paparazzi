#ifdef USE_SIM_UART


#include "simUart.h"
// #include "device_uart.h"
#include <string.h> //required for memcpy


// #define _SIM_UART_TRACES_

#ifdef _SIM_UART_TRACES_
#include <stdio.h>
#define SIM_UART_TRACE(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#else
#define SIM_UART_TRACE(...)
#endif

struct uart_periph sim_uart = INITIALIZED_UART_PERIPH("SIM_UART");
//AUXILIAR TEST PARAMETERS
uint8_t counter = 0;
bool_t loop = TRUE;

// FUNCTION DECLARATION -------------------------------------------------------
// neither uart.c nor uart_arch.c are compiled and thus some functions have to be defined here
  // uart.c functions
void uart_periph_init(struct uart_periph* p) __attribute__((alias("copy_uart_periph_init")));
void copy_uart_periph_init(struct uart_periph* p);
  // arch_uart.c functions
void uart_sendMessage(struct uart_periph *uart, uint8_t idx, void* trans, uint8_t priority) __attribute__((alias("fake_uart_sendMessage")));
void fake_uart_sendMessage(struct uart_periph *p, uint8_t idx, void* transaction, uint8_t priority);
void uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud) __attribute__((alias("fake_uart_periph_set_baudrate")));
void fake_uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud);

void fake_uart_ISR(struct uart_periph *p);


// FUNCTION DEFINITION --------------------------------------------------------
void copy_uart_periph_init(struct uart_periph* p) {
  //local copy of uart_init since uart.c is not compiled
  p->rx_insert_idx = 0;
  p->rx_extract_idx = 0;
  transmit_queue_init(&(p->tx_queue));
  p->tx_byte_idx = 0;
  p->tx_running = FALSE;
  p->ore = 0;
  p->ne_err = 0;
  p->fe_err = 0;
}

static inline void device_simUart_send_byte(struct uart_periph *p) {
  uint8_t data;
  uint16_t temp;

  //save data in local 'data' variable instead of transmit register...
  data = *((uint8_t*)(p->trans.data + p->tx_byte_idx++));
  SIM_UART_TRACE("\tsimUart: send_byte:  SENDING BYTE %u (%u)...\n", p->tx_byte_idx, data);
  if(loop) {
    p->rx_buf[p->rx_insert_idx] = data;
    temp = (p->rx_insert_idx + 1) % UART_RX_BUFFER_SIZE;
    if(temp != p->rx_extract_idx)
      p->rx_insert_idx = temp; // update insert index
  }
}

void fake_uart_sendMessage(struct uart_periph *p, uint8_t idx, void* transaction, uint8_t priority) {
  //Insert message in transmit queue
  SIM_UART_TRACE("\n\tsimUart: fake_uart_sendMessage:  INSERTING SLOT %u IN TRANSMIT QUEUE...\n", idx);
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
    SIM_UART_TRACE("\n\tsimUart: fake_uart_sendMessage:  STARTING TRANSMITION...\n");
    if( transmit_queue_extract_slot(&(p->tx_queue), &(p->trans_p)) ){
      memcpy(&(p->trans), (p->trans_p), sizeof(struct uart_transaction));
      SIM_UART_TRACE("\tsimUart: fake_uart_sendMessage:  SENDING MESSAGE (message length = %u)\n", p->trans.length);
      // set running flag and get a message to send
      p->tx_running = 1;
      p->tx_byte_idx = 0;
      device_simUart_send_byte(p);
      
      fake_uart_ISR(p);
    }
  }
}

void fake_uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud) {
  (void) p;
  (void) baud;
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
//       SIM_UART_TRACE("\tsimUart: fake_uart_ISR:  STOP SENDING\n");
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
        SIM_UART_TRACE("\tsimUart: fake_uart_ISR:  NO MORE DATA TO SEND\n");
        p->tx_running = 0;       // clear running flag
        return;
      } 
    }       
  }
}


#endif /* USE_SIM_UART */

