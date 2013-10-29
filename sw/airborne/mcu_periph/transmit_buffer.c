#include "transmit_buffer.h"

/* NOTE: this file works exchanging slot indexes instead of slot pointers
 * in order to avoid the case of a user passing pointers to local slots.
 * In that way transmit_buffer never goes out of its dedicated memory.
 * NOTE 2: different functions could attemp to modify the buffer at same time.
 * The use of semaphores is required.
 */

#define _TRANSMIT_BUFFER_DEBUG_

#ifdef _TRANSMIT_BUFFER_DEBUG_
#include <stdio.h> //for printf. Remove after debugging!
#define TRACE    printf
//#define TRACE(fmt,...)    printf(fmt)
#define DEBUG_PRINT_BUFFER { \
  uint8_t i = tx_buff->first_output; \
  TRACE("\tBuffer order (slot):    \t{"); \
  while( i < TX_BUFF_NUM_SLOTS){ \
    TRACE("%u, ", i); \
    i = tx_buff->slot[i].next_slot; \
  } \
  TRACE("}\n"); \
\
  i = tx_buff->first_output; \
  TRACE("\tBuffer order (priority):\t{"); \
  while( i < TX_BUFF_NUM_SLOTS){ \
    TRACE("%u, ",tx_buff->slot[i].priority); \
    i = tx_buff->slot[i].next_slot; \
  } \
  TRACE("}\n"); \
}
#else
#define TRACE	// 
//#define TRACE(fmt,args...)
#define DEBUG_PRINT_BUFFER {}
#endif //_TRANSMIT_BUFFER_DEBUG_


/*
struct transmit_buffer tx_buff ={
  .slot[0 ... (TX_BUFF_NUM_SLOTS-1)].length = 0,
  .slot[0 ... (TX_BUFF_NUM_SLOTS-1)].status = ST_FREE,
  .slot[0 ... (TX_BUFF_NUM_SLOTS-1)].priority = 0,
  .slot[0 ... (TX_BUFF_NUM_SLOTS-1)].next_slot = TX_BUFF_NUM_SLOTS,
  .first_output = TX_BUFF_NUM_SLOTS,
  .semaphore_get = 0,
  .semaphore_queue = 0,
  .pdg_action.action = NONE
};
*/


void tx_buffer_init(struct transmit_buffer *tx_buff) {
  for (uint8_t i = 0; i < TX_BUFF_NUM_SLOTS; i++){
    tx_buff->slot[i].length = 0;
    tx_buff->slot[i].status = ST_FREE;
    tx_buff->slot[i].priority = 0;
    tx_buff->slot[i].next_slot = TX_BUFF_NUM_SLOTS;
  }
  tx_buff->first_output = TX_BUFF_NUM_SLOTS;
  tx_buff->semaphore_get = 0;
  tx_buff->semaphore_queue = 0;
  tx_buff->pdg_action.action = NONE;
}

uint8_t tx_buffer_get_slot(struct transmit_buffer *tx_buff, uint8_t length, uint8_t *idx){
  uint8_t slot_idx = 0;
  TRACE("\tbuffer_transmit: get_tx_slot:  msg_len = %u\n", length);
  
  //0- verify atomic operation
  if(++tx_buff->semaphore_get > 1) {
    //We are interrupting another 'get_tx_slot' call
    //this action cannot be saved as pending since it needs to return an id: so, just quit as action failed
    TRACE("\tbuffer_transmit: get_tx_slot:  INTERRUPTING --> QUIT\n"); DEBUG_PRINT_BUFFER;
    tx_buff->semaphore_get--;
    return TX_BUFF_FALSE;
  }

  //1- verify data length
  if( length > MAX_DATA_LENGTH){
    //message too long
    TRACE("\tbuffer_transmit: get_tx_slot:  LENGTH EXCEDED --> QUIT\n"); DEBUG_PRINT_BUFFER;
    tx_buff->semaphore_get--;	return TX_BUFF_FALSE;
  }

  //2- try to get a slot
  while(slot_idx < TX_BUFF_NUM_SLOTS && tx_buff->slot[slot_idx].status != ST_FREE)
    slot_idx++;
  if(slot_idx == TX_BUFF_NUM_SLOTS){
    //Buffer is full
    TRACE("\tbuffer_transmit: get_tx_slot:  BUFFER FULL --> QUIT\n"); DEBUG_PRINT_BUFFER;
    tx_buff->semaphore_get--;	return TX_BUFF_FALSE;
  }

  //3- Slot available!
  tx_buff->slot[slot_idx].status = ST_RESERVED;
  tx_buff->slot[slot_idx].length = length;
  *idx = slot_idx;
  TRACE("\tbuffer_transmit: get_tx_slot:  slot_id = %u\n", slot_idx);

  tx_buff->semaphore_get--;	return TX_BUFF_TRUE;
}

uint8_t * tx_buffer_get_slot_pointer(struct transmit_buffer *tx_buff, uint8_t idx){
  return &(tx_buff->slot[idx].buffer[0]);
}

void fill_buffer(struct transmit_buffer *tx_buff, uint8_t idx, uint8_t offset, uint8_t *origin, uint8_t length){
  //1- verify slot status
  if(tx_buff->slot[idx].status != ST_RESERVED){
    //POSSIBLE USAGE ERROR: what to do?
    return;
  }

  //2- verify data length
  if( (offset+length) > MAX_DATA_LENGTH){
    //message too long
    return;
  }

  memcpy(&(tx_buff->slot[idx].buffer[offset]), origin, length);
}
  
void insert_slot_in_queue(struct transmit_buffer *tx_buff, uint8_t idx){
  uint8_t queue_idx = tx_buff->first_output;
  uint8_t queue_idx_previous = TX_BUFF_NUM_SLOTS;
  uint8_t sec = 0; //security iteration counter (should be not necessary)

  //1- verify slot status
  if(tx_buff->slot[idx].status != ST_RESERVED){
    //POSSIBLE USAGE ERROR: what to do?
    TRACE("\tbuffer_transmit: insert_slot_in_queue:  SLOT IS BEING SENT, FREE OR ALREADY READY\n"); DEBUG_PRINT_BUFFER;
    return;
  }

  //2- insert slot in queue
  if (queue_idx >= TX_BUFF_NUM_SLOTS){
    //Buffer is free of READY and SENDING data.
    tx_buff->slot[idx].next_slot = TX_BUFF_NUM_SLOTS;
    tx_buff->slot[idx].status =	ST_READY;
    tx_buff->first_output =	idx;
    TRACE("\tbuffer_transmit: insert_slot_in_queue:  EMPTY BUFFER\n"); DEBUG_PRINT_BUFFER;
    return;
  }

  while(tx_buff->slot[queue_idx].status == ST_SENDING || (tx_buff->slot[queue_idx].status == ST_READY && tx_buff->slot[queue_idx].priority >= tx_buff->slot[idx].priority)){
    if(tx_buff->slot[queue_idx].next_slot < TX_BUFF_NUM_SLOTS){
      queue_idx_previous = queue_idx;
      queue_idx = tx_buff->slot[queue_idx].next_slot;
    }
    else{
      //End of queue. Insert here
      tx_buff->slot[idx].next_slot =	TX_BUFF_NUM_SLOTS;
      tx_buff->slot[idx].status =	ST_READY;
      tx_buff->slot[queue_idx].next_slot = idx;
      TRACE("\tbuffer_transmit: insert_slot_in_queue:  INSERTED AT THE END\n"); DEBUG_PRINT_BUFFER;
      return;
    }
    if(++sec == TX_BUFF_NUM_SLOTS){
      //Security counter. This should never happen!!!
      //If this happens there is an internal error in buffer management. FIX IT!
      tx_buffer_init(tx_buff);
      TRACE("\tbuffer_transmit: insert_slot_in_queue:  ERROR!!!! QUEUE MAKES A LOOP (INFINITE QUEUE)\n"); DEBUG_PRINT_BUFFER;
      return;
    }
  }

  if(tx_buff->slot[queue_idx].status != ST_READY){
    //If this happens there is an internal error in buffer management. FIX IT!
    tx_buffer_init(tx_buff);
    TRACE("\tbuffer_transmit: insert_slot_in_queue:  ERROR!!!! BUFFER CONTAINS A NON READY MESSAGE\n"); DEBUG_PRINT_BUFFER;
    return;
  }
  
  //Insert here
  tx_buff->slot[idx].next_slot =		queue_idx;
  tx_buff->slot[idx].status =			ST_READY;
  if(queue_idx_previous < TX_BUFF_NUM_SLOTS)	tx_buff->slot[queue_idx_previous].next_slot = idx;
  else    					tx_buff->first_output =	idx;
  TRACE("\tbuffer_transmit: insert_slot_in_queue:  INSERTED IN THE MIDDLE\n"); DEBUG_PRINT_BUFFER;
}

void extract_slot_from_queue(struct transmit_buffer *tx_buff, uint8_t idx){
  uint8_t queue_idx = tx_buff->first_output;
  //1- find slot in queue
  //case A- queue is empty
  if (queue_idx >= TX_BUFF_NUM_SLOTS){
    return;
  }
  //case B- slot is the first element in queue
  if(queue_idx == idx){
    if(tx_buff->slot[queue_idx].status != ST_SENDING){
      tx_buff->first_output = tx_buff->slot[idx].next_slot;
      tx_buffer_free_slot(tx_buff, idx);
    }
    return;
  }
  //case C- slot is in the middle
  while(tx_buff->slot[queue_idx].next_slot < TX_BUFF_NUM_SLOTS){
    if(tx_buff->slot[queue_idx].next_slot == idx){
      tx_buff->slot[queue_idx].next_slot = tx_buff->slot[idx].next_slot;
      tx_buffer_free_slot(tx_buff, idx);
    }
  }
  //case D- slot is not in queue
}

void pending_action(struct transmit_buffer *tx_buff){
  TRACE("\tbuffer_transmit: pending_action:\n");
  switch(tx_buff->pdg_action.action){
    case ADD_QUEUE:
      insert_slot_in_queue(tx_buff, tx_buff->pdg_action.idx);
      break;
    case RMV_QUEUE:
    default:
      break;
  }
  tx_buff->pdg_action.action = NONE;
}

void try_insert_slot_in_queue(struct transmit_buffer *tx_buff, uint8_t idx, uint8_t priority){
  //0- verify slot idx
  if(idx >= TX_BUFF_NUM_SLOTS){
    //POSSIBLE USAGE ERROR: what to do?
    return;
  }

  //1- verify slot status
  if(tx_buff->slot[idx].status != ST_RESERVED){
    //POSSIBLE USAGE ERROR: what to do?
    return;
  }

  //2- set priority
  tx_buff->slot[idx].priority = priority;

  //3- verify atomic operation
  if(++tx_buff->semaphore_queue > 1){
    //We are interrupting another function which also attemps to modify the queue.
    //Save this action as pending and quit
    tx_buff->pdg_action.action = ADD_QUEUE;
    tx_buff->pdg_action.idx = idx;
    TRACE("\tbuffer_transmit: try_insert_slot_in_queue:  BUFFER BUSY. ACTION SAVED AS PENDING\n");
    tx_buff->semaphore_queue--;	return;
  }

  //4- insert slot in queue
  insert_slot_in_queue(tx_buff, idx);
  
  //5- execute pending action on queue
  pending_action(tx_buff);

  tx_buff->semaphore_queue--;
}

void try_extract_slot_from_queue(struct transmit_buffer *tx_buff, uint8_t idx){
  //0- verify slot idx
  if(idx >= TX_BUFF_NUM_SLOTS){
    //POSSIBLE USAGE ERROR: what to do?
    return;
  }

  //1- verify atomic operation
  if(++tx_buff->semaphore_queue > 1){
    //We are interrupting another function which also attemps to modify the queue.
    //Save this action as pending and quit
    tx_buff->pdg_action.action = RMV_QUEUE;
    tx_buff->pdg_action.idx = idx;
    TRACE("\tbuffer_transmit: try_extract_slot_from_queue:  BUFFER BUSY. ACTION SAVED AS PENDING\n");
    tx_buff->semaphore_queue--;	return;
  }

  //2- extract slot from queue
  extract_slot_from_queue(tx_buff, idx);
  
  //3- execute pending action on queue
  pending_action(tx_buff);

  tx_buff->semaphore_queue--;
}

/* CAUTION: slot must be out from Tx queue before setting it as free */
void tx_buffer_free_slot(struct transmit_buffer *tx_buff, uint8_t idx) {
  tx_buff->slot[idx].length = 0;
  tx_buff->slot[idx].status = ST_FREE;
  tx_buff->slot[idx].priority = 0;
  tx_buff->slot[idx].next_slot = TX_BUFF_NUM_SLOTS;
}
