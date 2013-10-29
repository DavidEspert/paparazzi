#include "device.h"
#include "i2c.h"
#include "transmit_buffer.h"

#include <stdio.h> //for printf. Remove after debugging!
#define TRACE    printf
//#define TRACE    //


// I2C ------------------------------------------------------------------------
void dev_I2C_sendMessage(struct i2c_periph *i2c, struct transmit_buffer *tx_buff, uint8_t idx, uint8_t priority) {
  uint8_t temp;
  //Insert message in transmit queue
  try_insert_slot_in_queue(tx_buff, idx, priority);

  //This is bullshit. 
  while(tx_buff->first_output < TX_BUFF_NUM_SLOTS){
    tx_buff->slot[tx_buff->first_output].status = ST_SENDING; //freeze this message as first output
      temp = tx_buff->first_output;
      tx_buff->first_output = tx_buff->slot[tx_buff->first_output].next_slot;
      tx_buffer_free_slot(tx_buff, temp);
      TRACE("\tdevice: dev_UART_sendMessage:  message from slot %u passed to UART\n", tx_buff->first_output);
  }
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

