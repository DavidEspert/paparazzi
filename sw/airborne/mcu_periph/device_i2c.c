#include "device.h"
#include "i2c.h"
#include "transmit_buffer.h"

#include <stdio.h> //for printf. Remove after debugging!
#define TRACE    printf
//#define TRACE    //

uint8_t counter = 0;

// I2C ------------------------------------------------------------------------
void dev_I2C_sendMessage(struct i2c_periph *i2c, struct transmit_buffer *tx_buff, uint8_t idx, uint8_t priority) {
  uint8_t first_send;
  //Insert message in transmit queue
  try_insert_slot_in_queue(tx_buff, idx, priority);
  
  /* TEST 0: Send nothing */

  /* TEST 1: Send everything 
  first_send = tx_buff->first_send;
  tx_buff->slot[first_send].status = ST_SENDING; //freeze this message as first output
  tx_buff->slot[first_send].status = ST_READY;
  TRACE("\tdevice: dev_I2C_sendMessage:  message from slot %u passed to I2C\n", first_send);
  tx_buffer_try_free_slot(tx_buff, first_send); */

  /* TEST 2: Send 1msg every 4msgs */
  /* message 0 has priority 0 (8bytes)
   * message 1 has priority 1 (21bytes)
   * message 2 has priority 0 (8bytes)
   * message 3 has priority 1 (21bytes)
   * After that the order in memory must be:   0,1,2,3
   * After that the order in TX queue must be: 1,3,0,2
   * First message is send: 1
   * message 4 has priority 0 (8bytes)
   * After that the order in memory must be:   0,4,2,3
   * After that the order in TX queue must be: 3,0,2,4
   */ 
  if(++counter == 4) {
    counter = 0;
    first_send = tx_buff->first_send;
    tx_buff->slot[first_send].status = ST_SENDING; //freeze this message as first output
    tx_buff->slot[first_send].status = ST_READY;
    TRACE("\n\tdevice: dev_I2C_sendMessage:  MESSAGE FROM SLOT %u HAS BEEN SENT\n", first_send);
    tx_buffer_try_free_slot(tx_buff, first_send);
  }

/*
  //This is bullshit.
  first_send = tx_buff->first_send;
//  while(first_send < TX_BUFF_NUM_SLOTS){
    tx_buff->slot[first_send].status = ST_SENDING; //freeze this message as first output
    tx_buff->slot[first_send].status = ST_READY;
    TRACE("\tdevice: dev_I2C_sendMessage:  message from slot %u passed to I2C\n", first_send);
    tx_buffer_try_free_slot(tx_buff, first_send);
//    first_send = tx_buff->first_send;
//  }
*/
}



#ifdef USE_I2C0
#endif /* USE_I2C0 */

#ifdef USE_I2C1
struct transmit_buffer i2c1_tx_buff = INITIALIZED_TX_BUFFER;

bool_t dev_I2C1_checkFreeSpace(uint8_t length)					{return TRUE;}
void dev_I2C1_transmit(uint8_t data)						{}
uint8_t dev_I2C1_get_tx_slot(uint8_t length, uint8_t *idx)			{ return tx_buffer_get_slot(&i2c1_tx_buff, length, idx); }
uint8_t * dev_I2C1_get_buff_pointer(uint8_t idx)				{ return tx_buffer_get_slot_pointer(&i2c1_tx_buff, idx); }
void dev_I2C1_sendMessage(uint8_t idx, uint8_t priority)			{ dev_I2C_sendMessage(&i2c1, &i2c1_tx_buff, idx, priority); }


struct device dev_I2C1 = {
//  .init = ,
  .checkFreeSpace  =	&dev_I2C1_checkFreeSpace,
  .transmit =		&dev_I2C1_transmit,
  .get_tx_slot =	&dev_I2C1_get_tx_slot,
  .get_buff_pointer =	&dev_I2C1_get_buff_pointer,
  .sendMessage =	&dev_I2C1_sendMessage
};

#endif /* USE_I2C1 */

#ifdef USE_I2C2
#endif /* USE_I2C2 */

#ifdef USE_I2C3
#endif /* USE_I2C3 */

