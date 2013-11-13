#ifdef USE_SIM_UART


#include "device.h"
#include "transmit_buffer.h"


// #define _I2C_DEBUG_

#ifdef _I2C_DEBUG_
#include <stdio.h>
#define TRACE(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#else
#define TRACE(...)
#endif


uint8_t counter = 0;
void fake_ISR(struct transmit_buffer *tx_buff);

uint8_t tx_running = 0;
uint8_t first_send;
uint8_t tx_byte_idx = 0;

// SIM ------------------------------------------------------------------------
void dev_SIM_sendMessage_buff(struct transmit_buffer *tx_buff, uint8_t idx, uint8_t priority) {
  uint8_t data;

  //Insert message in transmit queue
  TRACE("\n\tdevice_i2c: dev_SIM_sendMessage:  INSERTING SLOT %u IN TRANSMIT QUEUE...\n", idx);
  tx_buffer_try_insert_slot(tx_buff, idx, priority);

  /* TEST 0: Send nothing */
//   return;

  /* TEST 1: Send everything */

  /* TEST 2: Send 1msg every 4msgs */
//   if(++counter == 4)    counter = 0;
//   else                  return;


  // check if in process of sending data
  if (tx_running) {
    // Nothing to do
  } else {
    if( tx_buffer_get_slot_send(tx_buff, &first_send) ){
      TRACE("\n\tdevice_i2c: dev_SIM_sendMessage:  SENDING MESSAGE %u\n", first_send);
      // set running flag and get a message to send
      tx_running = 1;
      tx_byte_idx = 0;
      data = tx_buffer_get_slot_data(tx_buff, first_send, tx_byte_idx++);
      TRACE("\tdevice_i2c: dev_SIM_sendMessage:  sending byte %u (%u)...\n", tx_byte_idx, data);
      
      fake_ISR(tx_buff);
    }
  }
}


void fake_ISR(struct transmit_buffer *tx_buff) {
  uint8_t msg_length;
  uint8_t data;

  msg_length = tx_buff->slot[first_send].length;
  while(1) {
    // check if more data to send in actual message
    if(tx_byte_idx < msg_length) {
      data = tx_buffer_get_slot_data(tx_buff, first_send, tx_byte_idx++);
      TRACE("\tdevice_i2c: fake_ISR:  sending byte %u (%u)...\n", tx_byte_idx, data);
    }
    else{
      //message ended. Free space
      tx_buffer_try_free_slot(tx_buff, first_send);
      // check if there is a new message
      if( tx_buffer_get_slot_send(tx_buff, &first_send) ){
        msg_length = tx_buff->slot[first_send].length;
        tx_byte_idx = 0;
        data = tx_buffer_get_slot_data(tx_buff, first_send, tx_byte_idx++);
        TRACE("\tdevice_i2c: fake_ISR:  sending byte %u (%u)...\n", tx_byte_idx, data);
      }
      else{
        tx_running = 0;       // clear running flag
        return;
      } 
    }       
  }
}


struct transmit_buffer sim_tx_buff = INITIALIZED_TX_BUFFER;

bool_t dev_SIM_checkFreeSpace(uint8_t length, uint8_t *idx)                                    { return tx_buffer_get_slot_mem(&sim_tx_buff, length, idx); }
uint8_t * dev_SIM_get_buff_pointer(uint8_t idx)                                                { return tx_buffer_get_slot_pointer(&sim_tx_buff, idx); }
void dev_SIM_packMessage(uint8_t idx, uint8_t offset, uint8_t *origin, uint8_t length)         { tx_buffer_fill(&sim_tx_buff, idx, offset, origin, length); }
void dev_SIM_sendMessage(uint8_t idx, uint8_t priority)                                        { dev_SIM_sendMessage_buff(&sim_tx_buff, idx, priority); }


struct device dev_SIM_UART = {
  .checkFreeSpace =     &dev_SIM_checkFreeSpace,
  .get_buff_pointer =   &dev_SIM_get_buff_pointer,
  .packMessage =        &dev_SIM_packMessage,
  .sendMessage =        &dev_SIM_sendMessage
};

#endif /* USE_SIM_UART */

