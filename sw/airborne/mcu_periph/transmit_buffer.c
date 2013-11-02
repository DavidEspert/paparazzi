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
#define DEBUG_PRINT_BUFFER_SEND { \
  uint8_t i = tx_buff->first_send; \
  TRACE("\tBuffer SEND order (slot):    \t{"); \
  while( i < TX_BUFF_NUM_SLOTS){ \
    TRACE("%3u, ", i); \
    i = tx_buff->slot[i].next_send; \
  } \
  TRACE("}\n"); \
\
  i = tx_buff->first_send; \
  TRACE("\tBuffer SEND order (priority):\t{"); \
  while( i < TX_BUFF_NUM_SLOTS){ \
    TRACE("%3u, ",tx_buff->slot[i].priority); \
    i = tx_buff->slot[i].next_send; \
  } \
  TRACE("}\n"); \
}

#define DEBUG_PRINT_BUFFER_MEM { \
  uint8_t i = tx_buff->first_mem; \
  TRACE("\tBuffer MEM  order (slot):    \t{"); \
  while( i < TX_BUFF_NUM_SLOTS){ \
    TRACE("%3u, ", i); \
    i = tx_buff->slot[i].next_mem; \
  } \
  TRACE("}\n"); \
\
  i = tx_buff->first_mem; \
  TRACE("\tBuffer MEM  order (init):    \t{"); \
  while( i < TX_BUFF_NUM_SLOTS){ \
    TRACE("%3u, ",tx_buff->slot[i].init); \
    i = tx_buff->slot[i].next_mem; \
  } \
  TRACE("}\n"); \
\
  i = tx_buff->first_mem; \
  TRACE("\tBuffer MEM  order (end):     \t{"); \
  while( i < TX_BUFF_NUM_SLOTS){ \
    TRACE("%3u, ",(tx_buff->slot[i].init + tx_buff->slot[i].length -1)); \
    i = tx_buff->slot[i].next_mem; \
  } \
  TRACE("}\n"); \
}

#else
#define TRACE	// 
//#define TRACE(fmt,args...)
#define DEBUG_PRINT_BUFFER_SEND {}
#define DEBUG_PRINT_BUFFER_MEM {}
#endif //_TRANSMIT_BUFFER_DEBUG_


/*
struct transmit_buffer tx_buff ={
  .slot[0 ... (TX_BUFF_NUM_SLOTS-1)] = { .status = ST_FREE, .init = 0, .length = 0, .priority = 0, .next_send = TX_BUFF_NUM_SLOTS, .next_mem = TX_BUFF_NUM_SLOTS}, \
  .first_send = TX_BUFF_NUM_SLOTS,
  .first_mem = TX_BUFF_NUM_SLOTS,
  .semaphore_get = 0,
  .semaphore_queue = 0,
  .pdg_action.action = NONE
};
*/

bool_t tx_buffer_get_slot(struct transmit_buffer *tx_buff, uint8_t length, uint8_t *idx){
  uint8_t slot_idx = 0;
  uint8_t slot_idx2 = tx_buff->first_mem;
//  uint8_t sec = 0; //security iteration counter (should be not necessary)
  struct T_candidate {
    uint8_t init;
    uint8_t length;
    uint8_t  prev_mem;
    uint8_t  next_mem;
  } candidate = { BUFFER_LENGTH, BUFFER_LENGTH, TX_BUFF_NUM_SLOTS, TX_BUFF_NUM_SLOTS};

  //0- verify atomic operation
  if(++tx_buff->semaphore_get > 1) {
    //We are interrupting another 'get_tx_slot' call
    //this action cannot be saved as pending since it has to return an id: so, just quit as action failed
    TRACE("\tbuffer_transmit: get_tx_slot:  INTERRUPTING --> QUIT\n"); DEBUG_PRINT_BUFFER_MEM; DEBUG_PRINT_BUFFER_SEND;
    tx_buff->semaphore_get--;
    return FALSE;
  }

  //1- verify data length
/* NOTE: This verification has no sense with 'length' of uint8_t type and BUFFER_LENGTH = 255.
 *       Using uint16_t would have sense but still it would not be necessary. 
    if(length > BUFFER_LENGTH){ 
    //(this step is not necessary but it saves time in case of 'length > BUFFER_LENGTH'
    TRACE("\tbuffer_transmit: get_tx_slot:  LENGTH EXCEEDED --> QUIT\n"); DEBUG_PRINT_BUFFER_MEM; DEBUG_PRINT_BUFFER_SEND;
    tx_buff->semaphore_get--;	return FALSE;
  }
*/
  //2- try to get a slot
  while(slot_idx < TX_BUFF_NUM_SLOTS && tx_buff->slot[slot_idx].status != ST_FREE)
    slot_idx++;

  if(slot_idx == TX_BUFF_NUM_SLOTS){
    //Buffer is full
    TRACE("\tbuffer_transmit: get_tx_slot:  ANY SLOT AVAILABLE --> QUIT\n"); DEBUG_PRINT_BUFFER_MEM; DEBUG_PRINT_BUFFER_SEND;
    tx_buff->semaphore_get--;	return FALSE;
  }

  // slot available!
  // That is: there is a free slot struct that could manage the message but it
  // must be checked if there is memory enough for this message

  //3- look for memory space
  //3.1- There is any slot in memory
  if(slot_idx2 == TX_BUFF_NUM_SLOTS) {
    if(tx_buff->first_send != TX_BUFF_NUM_SLOTS) {
      //If this happens there is an internal error in buffer management. FIX IT!
      //Error. first_mem indicates empty buffer but first_send indicates non-empty buffer!
      tx_buffer_init(tx_buff);
      TRACE("\tbuffer_transmit: get_tx_slot:\n\t\tERROR!!!! Unmatched 'firt_mem - first_send')\n"); DEBUG_PRINT_BUFFER_MEM; DEBUG_PRINT_BUFFER_SEND;
      return FALSE;
    }
    candidate.init =		0;
    candidate.length =		BUFFER_LENGTH;
    candidate.prev_mem =	TX_BUFF_NUM_SLOTS;
    candidate.next_mem =	TX_BUFF_NUM_SLOTS;
    //TRACE("\tbuffer_transmit: get_tx_slot:  candidate found : init = %u, length available = %u, previous slot = %u, next slot = %u\n", candidate.init, candidate.length, candidate.prev_mem, candidate.next_mem);
    goto end;
  }
  //3.2- There is a hole at the beginning
  if(tx_buff->slot[slot_idx2].init >= length) {
    candidate.init =		0;
    candidate.length =		tx_buff->slot[slot_idx2].init;
    candidate.prev_mem =	TX_BUFF_NUM_SLOTS;
    candidate.next_mem =	slot_idx2;// = tx_buff->first_mem;
    //TRACE("\tbuffer_transmit: get_tx_slot:  candidate found : init = %u, length available = %u, previous slot = %u, next slot = %u\n", candidate.init, candidate.length, candidate.prev_mem, candidate.next_mem);
  }
  //3.3- Check holes after the existing slots
  while (slot_idx2 < TX_BUFF_NUM_SLOTS) {
    uint8_t  next_mem =  tx_buff->slot[slot_idx2].next_mem;
    uint8_t hole_init = tx_buff->slot[slot_idx2].init + tx_buff->slot[slot_idx2].length;
    uint8_t hole_length;
    if(next_mem < TX_BUFF_NUM_SLOTS)	hole_length = tx_buff->slot[next_mem].init - hole_init;
    else				hole_length = BUFFER_LENGTH - hole_init;
    //look for the smallest suitable hole in memory. Leave bigger ones for bigger messages
    if (hole_length >= length && hole_length < candidate.length) {
//    if (hole_length >= length) {
//      TRACE("\tbuffer_transmit: get_tx_slot:  candidate found : init = %u, length available = %u, previous slot = %u, next slot = %u\n", hole_init, hole_length, slot_idx2, next_mem);
//      if(hole_length < candidate.length) {
	candidate.init =		hole_init;
	candidate.length =	hole_length;
	candidate.prev_mem =	slot_idx2;
	candidate.next_mem =	next_mem;
//      }
    }
    slot_idx2 = next_mem;

/*    if(++sec == TX_BUFF_NUM_SLOTS){
      //Security counter. This should never happen!!!
      //If this happens there is an internal error in buffer management. FIX IT!
      tx_buffer_init(tx_buff);
      TRACE("\tbuffer_transmit: get_tx_slot:  ERROR!!!! 'next_mem' MAKES AN INFINITE LOOP\n"); DEBUG_PRINT_BUFFER_MEM; DEBUG_PRINT_BUFFER_SEND;
      return FALSE;
    }*/
  }

end:
  if(candidate.init < BUFFER_LENGTH) {
    tx_buff->slot[slot_idx].status =	ST_RESERVED;
    tx_buff->slot[slot_idx].init =	candidate.init;
    tx_buff->slot[slot_idx].length =	length;
    if(candidate.prev_mem < TX_BUFF_NUM_SLOTS)
      tx_buff->slot[candidate.prev_mem].next_mem = slot_idx;
    else
      tx_buff->first_mem =		slot_idx;
    tx_buff->slot[slot_idx].next_mem =	candidate.next_mem;

    *idx = slot_idx;
    TRACE("\tbuffer_transmit: get_tx_slot:  ASSIGNED SLOT %u (MESSAGE LENGTH = %u)\n", slot_idx, length); DEBUG_PRINT_BUFFER_MEM;
    tx_buff->semaphore_get--;	return TRUE;
  }
  else {
    TRACE("\tbuffer_transmit: get_tx_slot:  NO MEMORY AVAILABLE (MESSAGE LENGTH = %u) --> QUIT\n", length); DEBUG_PRINT_BUFFER_MEM; DEBUG_PRINT_BUFFER_SEND;
    tx_buff->semaphore_get--;	return FALSE;
  }
}

uint8_t * tx_buffer_get_slot_pointer(struct transmit_buffer *tx_buff, uint8_t idx){
//  if(idx >= TX_BUFF_NUM_SLOTS) return NULL;
  return &(tx_buff->buffer[(tx_buff->slot[idx].init)]);
}

void fill_buffer(struct transmit_buffer *tx_buff, uint8_t idx, uint8_t offset, uint8_t *origin, uint8_t length){
  //0- verify slot index
  //if(idx >= TX_BUFF_NUM_SLOTS) return;

  //1- verify slot status
  if(tx_buff->slot[idx].status != ST_RESERVED){
    //POSSIBLE USAGE ERROR: what to do?
    return;
  }

  //2- verify data length
  if( (offset+length) > tx_buff->slot[idx].length){
    //message too long
    return;
  }

  memcpy(&(tx_buff->buffer[(tx_buff->slot[idx].init + offset)]), origin, length);
}
  
void insert_slot_in_queue(struct transmit_buffer *tx_buff, uint8_t idx){
  uint8_t queue_idx = tx_buff->first_send;
  uint8_t queue_idx_previous = TX_BUFF_NUM_SLOTS;
//  uint8_t sec = 0; //security iteration counter (should be not necessary)

  //0- verify slot index
  //if(idx >= TX_BUFF_NUM_SLOTS) return;

  //1- verify slot status
  if(tx_buff->slot[idx].status != ST_RESERVED){
    //POSSIBLE USAGE ERROR: what to do?
    TRACE("\tbuffer_transmit: insert_slot_in_queue:  SLOT %u IS BEING SENT, FREE OR ALREADY READY\n", idx); DEBUG_PRINT_BUFFER_SEND;
    return;
  }

  //2- insert slot in queue
  if (queue_idx >= TX_BUFF_NUM_SLOTS){
    //Buffer is free of READY and SENDING data.
    tx_buff->slot[idx].next_send =		TX_BUFF_NUM_SLOTS;
    tx_buff->slot[idx].status =			ST_READY;
    tx_buff->first_send =			idx;
    TRACE("\tbuffer_transmit: insert_slot_in_queue:  EMPTY QUEUE. SLOT %u INSERTED IN FIRST PLACE (PRIORITY = %u)\n", idx, tx_buff->slot[idx].priority); DEBUG_PRINT_BUFFER_SEND;
    return;
  }

  while(tx_buff->slot[queue_idx].status == ST_SENDING || (tx_buff->slot[queue_idx].status == ST_READY && tx_buff->slot[queue_idx].priority >= tx_buff->slot[idx].priority)){
    if(tx_buff->slot[queue_idx].next_send < TX_BUFF_NUM_SLOTS){
      queue_idx_previous = queue_idx;
      queue_idx = tx_buff->slot[queue_idx].next_send;
    }
    else{
      //End of queue. Insert here
      tx_buff->slot[idx].next_send =		TX_BUFF_NUM_SLOTS;
      tx_buff->slot[idx].status =		ST_READY;
      tx_buff->slot[queue_idx].next_send =	idx;
      TRACE("\tbuffer_transmit: insert_slot_in_queue:  SLOT %u INSERTED AT THE END (PRIORITY = %u)\n", idx, tx_buff->slot[idx].priority); DEBUG_PRINT_BUFFER_SEND;
      return;
    }
/*    if(++sec == TX_BUFF_NUM_SLOTS){
      //Security counter. This should never happen!!!
      //If this happens there is an internal error in buffer management. FIX IT!
      tx_buffer_init(tx_buff);
      TRACE("\tbuffer_transmit: insert_slot_in_queue:  ERROR!!!! QUEUE MAKES A LOOP (INFINITE QUEUE)\n"); DEBUG_PRINT_BUFFER_SEND;
      return;
    }*/
  }

  if(tx_buff->slot[queue_idx].status != ST_READY){
    //If this happens there is an internal error in buffer management. FIX IT!
    tx_buffer_init(tx_buff);
    TRACE("\tbuffer_transmit: insert_slot_in_queue:  ERROR!!!! QUEUE CONTAINS A NON READY MESSAGE\n"); DEBUG_PRINT_BUFFER_SEND;
    return;
  }
  
  //Insert here
  tx_buff->slot[idx].next_send =		queue_idx;
  tx_buff->slot[idx].status =			ST_READY;
  if(queue_idx_previous < TX_BUFF_NUM_SLOTS) {
    tx_buff->slot[queue_idx_previous].next_send = idx;
    TRACE("\tbuffer_transmit: insert_slot_in_queue:  SLOT %u INSERTED IN THE MIDDLE (PRIORITY = %u)\n", idx, tx_buff->slot[idx].priority); DEBUG_PRINT_BUFFER_SEND;
  }
  else {
    tx_buff->first_send =			idx;
    TRACE("\tbuffer_transmit: insert_slot_in_queue:  SLOT %u INSERTED IN FIRST PLACE (PRIORITY = %u)\n", idx, tx_buff->slot[idx].priority); DEBUG_PRINT_BUFFER_SEND;
  }
}

bool_t extract_slot_from_queue(struct transmit_buffer *tx_buff, uint8_t idx){
  uint8_t queue_idx = tx_buff->first_send;
//  uint8_t sec = 0; //security iteration counter (should be not necessary)

  //0- verify slot index
  //if(idx >= TX_BUFF_NUM_SLOTS) return;

  //1- verify slot status
  if(tx_buff->slot[idx].status == ST_SENDING){
    TRACE("\tbuffer_transmit: extract_slot_from_queue:  SLOT %u CANNOT BE EXTRACTED FROM QUEUE (IT IS BEING SENT)\n", idx); DEBUG_PRINT_BUFFER_SEND;
    return FALSE;
  }

  //2- find slot in queue
  //case A- queue is empty
  if (queue_idx >= TX_BUFF_NUM_SLOTS){
    return TRUE;
  }
  //case B- slot is the first element in queue
  if(queue_idx == idx){
    tx_buff->first_send = tx_buff->slot[idx].next_send;
    goto end;
  }
  //case C- slot is in the middle
  while(tx_buff->slot[queue_idx].next_send < TX_BUFF_NUM_SLOTS){
    if(tx_buff->slot[queue_idx].next_send == idx){
      tx_buff->slot[queue_idx].next_send = tx_buff->slot[idx].next_send;
      goto end;
    }
    queue_idx = tx_buff->slot[queue_idx].next_send;

/*    if(++sec == TX_BUFF_NUM_SLOTS){
      //Security counter. This should never happen!!!
      //If this happens there is an internal error in buffer management. FIX IT!
      tx_buffer_init(tx_buff);
      TRACE("\tbuffer_transmit: extract_slot_from_queue:  ERROR!!!! QUEUE MAKES A LOOP (INFINITE QUEUE)\n"); DEBUG_PRINT_BUFFER_SEND;
      return;
    }*/
  }
  //case D- slot is not in queue
end:
  TRACE("\tbuffer_transmit: extract_slot_from_queue:  SLOT %u EXTRACTED FROM QUEUE\n", idx); DEBUG_PRINT_BUFFER_SEND;
  tx_buff->slot[idx].next_send = TX_BUFF_NUM_SLOTS;
  return TRUE;
}

/* CAUTION: slot must be out from Tx queue before setting it as free */
void tx_buffer_free_slot(struct transmit_buffer *tx_buff, uint8_t idx){
  //this function is called after 'extract_slot_from_queue'. Thus, since slot
  //'idx' is no longer in transmit queue, we just must take care of memory.
  uint8_t slot_idx = tx_buff->first_mem;
//  uint8_t sec = 0; //security iteration counter (should be not necessary)

  //0- verify slot index
  //if(idx >= TX_BUFF_NUM_SLOTS) return;

  //1- verify slot status
  //slot 'idx' is not in queue --> not ST_SENDING --> no problem

  //2- find slot in memory
  //case A- memory is empty
  if (slot_idx >= TX_BUFF_NUM_SLOTS){
    goto end;
  }
  //case B- slot is the first element in memory
  if(slot_idx == idx){
    tx_buff->first_mem = tx_buff->slot[idx].next_mem;
    goto end;
  }
  //case C- slot is in the middle
  while(tx_buff->slot[slot_idx].next_mem < TX_BUFF_NUM_SLOTS){
    if(tx_buff->slot[slot_idx].next_mem == idx){
      tx_buff->slot[slot_idx].next_mem = tx_buff->slot[idx].next_mem;
      goto end;
    }
    slot_idx = tx_buff->slot[slot_idx].next_mem;

/*    if(++sec == TX_BUFF_NUM_SLOTS){
      //Security counter. This should never happen!!!
      //If this happens there is an internal error in buffer management. FIX IT!
      tx_buffer_init(tx_buff);
      TRACE("\tbuffer_transmit: tx_buffer_free_slot:  ERROR!!!! MEMORY MAKES A LOOP (INFINITE MEMORY)\n"); DEBUG_PRINT_BUFFER_MEM;
      return;
    }*/
  }
  //case D- slot is already free
end:
  TRACE("\tbuffer_transmit: tx_buffer_free_slot:  SLOT %u EXTRACTED FROM MEMORY\n", idx); DEBUG_PRINT_BUFFER_MEM;
  tx_buff->slot[idx].init =		0;
  tx_buff->slot[idx].length =		0;
  tx_buff->slot[idx].priority =		0;
  tx_buff->slot[idx].next_send =	TX_BUFF_NUM_SLOTS;
  tx_buff->slot[idx].next_mem =		TX_BUFF_NUM_SLOTS;
  tx_buff->slot[idx].status =		ST_FREE;
}

void pending_action(struct transmit_buffer *tx_buff){
  switch(tx_buff->pdg_action.action){
    case ADD_QUEUE:
      TRACE("\tbuffer_transmit: pending_action: ADD slot %u in QUEUE\n", tx_buff->pdg_action.idx);
      insert_slot_in_queue(tx_buff, tx_buff->pdg_action.idx);
      break;
    case RMV_QUEUE:
      TRACE("\tbuffer_transmit: pending_action: REMOVE slot %u from QUEUE\n", tx_buff->pdg_action.idx);
      extract_slot_from_queue(tx_buff, tx_buff->pdg_action.idx);
      break;
    case FREE_SLOT:
      TRACE("\tbuffer_transmit: pending_action: FREE slot %u\n", tx_buff->pdg_action.idx);
      if(extract_slot_from_queue(tx_buff, tx_buff->pdg_action.idx))
	tx_buffer_free_slot(tx_buff, tx_buff->pdg_action.idx);
      break;
    default:
      break;
  }
  tx_buff->pdg_action.action = NONE;
}

void try_insert_slot_in_queue(struct transmit_buffer *tx_buff, uint8_t idx, uint8_t priority){
  //0- verify slot index
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

void tx_buffer_try_free_slot(struct transmit_buffer *tx_buff, uint8_t idx){
  //0- verify slot idx
  if(idx >= TX_BUFF_NUM_SLOTS){
    //POSSIBLE USAGE ERROR: what to do?
    return;
  }

  //1- verify atomic operation
  if(++tx_buff->semaphore_queue > 1){
    //We are interrupting another function which also attemps to modify the queue.
    //Save this action as pending and quit
    tx_buff->pdg_action.action = FREE_SLOT;
    tx_buff->pdg_action.idx = idx;
    TRACE("\tbuffer_transmit: tx_buffer_try_free_slot:  BUFFER BUSY. ACTION SAVED AS PENDING\n");
    tx_buff->semaphore_queue--;	return;
  }

  //2- extract slot from queue and set it free
//  extract_slot_from_queue(tx_buff, idx);
  if(extract_slot_from_queue(tx_buff, idx))
  { tx_buffer_free_slot(tx_buff, idx); }
  
  //3- execute pending action on queue
  pending_action(tx_buff);

  tx_buff->semaphore_queue--;
}

void tx_buffer_init(struct transmit_buffer *tx_buff) {
  for (uint8_t idx = 0; idx < TX_BUFF_NUM_SLOTS; idx++)
    tx_buffer_free_slot(tx_buff, idx);
  tx_buff->first_send =			TX_BUFF_NUM_SLOTS;
  tx_buff->first_mem =			TX_BUFF_NUM_SLOTS;
  tx_buff->semaphore_get =		0;
  tx_buff->semaphore_queue =		0;
  tx_buff->pdg_action.action =		NONE;
}

