#ifndef _TRANSMIT_BUFFER_H_
#define _TRANSMIT_BUFFER_H_

#include <stdint.h>
#include "std.h"	//required for bool_t definition

//Buffer status
  //enum tx_buff_status {ST_FREE, ST_RESERVED, ST_READY, ST_SENDING}
#define ST_FREE 1
#define ST_RESERVED 2
#define ST_READY 3
#define ST_SENDING 4
#define ST_SENT 5

//num of slots available (1 slot --> 1 message)
#define TX_BUFF_NUM_SLOTS	32
#define BUFFER_LENGTH		255

//identifiers for pending actions
#define NONE 0
#define ADD_QUEUE 1
#define	FREE_SLOT 2

//Declaration of initialized buffer: i.e. 'struct transmit_buffer tx_buff = INITIALIZED_TX_BUFFER;'
#define INITIALIZED_TX_BUFFER { \
  .slot[0 ... (TX_BUFF_NUM_SLOTS-1)] = { .status = ST_FREE, .init = 0, .length = 0, .priority = 0, .next_send = TX_BUFF_NUM_SLOTS, .next_mem = TX_BUFF_NUM_SLOTS}, \
  .first_send = TX_BUFF_NUM_SLOTS, \
  .first_mem = TX_BUFF_NUM_SLOTS, \
  .semaphore_get_mem = 0, \
  .semaphore_queue = 0, \
  .pdg_action.action = NONE \
}

struct pending_action {
  uint8_t	action;
  uint8_t	idx;
};

struct transmit_slot {
  uint8_t	status;         // message status {FREE, RESERVED, READY or SENDING}
  uint8_t	init;           // position of first message byte in buffer
  uint8_t	length;         // message length
  uint8_t	priority;       // message priority
  uint8_t	next_send;      // index of next slot to be sent
  uint8_t	next_mem;       // index of next slot in memory
//  transmit_slot *next_buff;  (use next_slot instead)
};

//#pragma DATA_ALIGN(256)
struct transmit_buffer {
  uint8_t		buffer[BUFFER_LENGTH];
  struct transmit_slot	slot[TX_BUFF_NUM_SLOTS];
  uint8_t		first_send;	//first slot to be send/sending
  uint8_t		first_mem;	//first slot in memory
  uint8_t		semaphore_get_mem;
  uint8_t		semaphore_queue;
  struct pending_action pdg_action;
};

//FILLING FUNCTIONS - (To be used by sender) ---------------------------------------------------
//Get a slot for sending data. If a slot is available, it will be marked as ST_RESERVED and 'idx' will be filled with its index.
bool_t tx_buffer_get_slot_mem(struct transmit_buffer *tx_buff, uint8_t length, uint8_t *idx);

//Put 'length' bytes, from 'origin' address, in the ST_RESERVED slot (with an offset).
void tx_buffer_fill(struct transmit_buffer *tx_buff, uint8_t idx, uint8_t offset, uint8_t *origin, uint8_t length);

//Insert a slot in transmit queue. Slot will be marked as ST_READY and insertion will be done according to the assigned priority.
void tx_buffer_try_insert_slot(struct transmit_buffer *tx_buff, uint8_t idx, uint8_t priority);



//FLUSHING FUNCTIONS ----------------------------------------------------
//  (To be used by sender...)
//If not marked as ST_SENDING, slot will be extracted from transmit queue and will remain in memory marked as ST_RESERVED
bool_t tx_buffer_try_extract_slot(struct transmit_buffer *tx_buff, uint8_t idx);

//If it is not marked as ST_SENDING, slot will be extracted from queue and set as free.
void tx_buffer_try_flush_slot(struct transmit_buffer *tx_buff, uint8_t idx);


//  (To be used by transmit device...)
//Mark slot as ST_SENT, extract it from queue and free it. This function works on messages marked as ST_SENDING
void tx_buffer_try_free_slot(struct transmit_buffer *tx_buff, uint8_t idx);

//Get next slot to be send
bool_t tx_buffer_get_slot_send(struct transmit_buffer *tx_buff, uint8_t *idx);



//OTHER- (To be used by both sender and transmit device) ---------------------------------------------------
//Initialize buffer.
void tx_buffer_init(struct transmit_buffer *tx_buff);

//Get initial address of slot's buffer.
uint8_t * tx_buffer_get_slot_pointer(struct transmit_buffer *tx_buff, uint8_t idx);
//bool_t tx_buffer_get_slot_pointer(struct transmit_buffer *tx_buff, uint8_t idx, uint8_t ** pointer);

//Get a byte from an specific slot.
uint8_t tx_buffer_get_slot_data(struct transmit_buffer *tx_buff, uint8_t slot_idx, uint8_t byte_idx);
//bool_t tx_buffer_get_slot_data(struct transmit_buffer *tx_buff, uint8_t slot_idx, uint8_t byte_idx, uint8_t *data);

#endif // _TRANSMIT_BUFFER_H_