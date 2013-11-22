#include "transmit_queue.h"

// Checks ensure that a given parameter 'slot index'
// corresponds to an existing and reserved slot in queue.
// PUBLIC  checks find possible user errors --> What to do with this? Always enabled? Just for Debug?
// PRIVATE checks find internal errors --> DISABLE AFTER DEVELOPMENT
#define _TRANSMIT_QUEUE_PUBLIC_CHECKS_
#define _TRANSMIT_QUEUE_PRIVATE_CHECKS_

// #define _TRANSMIT_QUEUE_TRACES_

#ifdef _TRANSMIT_QUEUE_TRACES_
#include <stdio.h>
#define TRACE(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#define DEBUG_PRINT_QUEUE { \
  uint8_t i = queue->first_send; \
  TRACE("\tTransmit queue order (slot):    \t{"); \
  while( i < TRANSMIT_QUEUE_LEN){ \
    TRACE("%3u, ", i); \
    i = queue->slot[i].next_send; \
  } \
  TRACE("}\n"); \
\
  i = queue->first_send; \
  TRACE("\tTransmit queue order (priority):\t{"); \
  while( i < TRANSMIT_QUEUE_LEN){ \
    TRACE("%3u, ",queue->slot[i].priority); \
    i = queue->slot[i].next_send; \
  } \
  TRACE("}\n"); \
}

#else
#define TRACE(...)
#define DEBUG_PRINT_QUEUE {}
#endif //_TRANSMIT_QUEUE_TRACES_

//Private functions
void transmit_queue_insert_slot_atomic(struct transmit_queue *queue, uint8_t idx);
void transmit_queue_pending_action(struct transmit_queue *queue);
#if defined (_TRANSMIT_QUEUE_PUBLIC_CHECKS_) || defined (_TRANSMIT_QUEUE_PRIVATE_CHECKS_)
bool_t transmit_queue_validate_slot_idx(struct transmit_queue *queue, uint8_t idx);
#endif

/* struct transmit_queue queue = INITIALIZED_TRANSMIT_QUEUE; */


//Public
void transmit_queue_init(struct transmit_queue *queue) {
  for (uint8_t idx = 0; idx < TRANSMIT_QUEUE_LEN; idx++) {
    queue->slot[idx].status =           ST_FREE;
    queue->slot[idx].data =             NULL;
    queue->slot[idx].priority =         0;
    queue->slot[idx].next_send =        TRANSMIT_QUEUE_LEN;
  }
  queue->first_send =                   TRANSMIT_QUEUE_LEN;
  queue->semaphore_get_slot =           0;
  queue->semaphore_queue =              0;
  queue->pdg_action.action =            NONE;
}

//Public
bool_t transmit_queue_check_free_space(struct transmit_queue *queue, uint8_t *idx){
  uint8_t slot_idx = 0;

  //0- verify atomic operation
  if(++queue->semaphore_get_slot > 1) {
    //We are interrupting another 'transmit_queue_check_free_space' call
    //this action cannot be saved as pending since it has to return an id: so, just quit as action failed
    TRACE("\ttransmit_queue: check_free_space:  INTERRUPTING --> QUIT\n");DEBUG_PRINT_QUEUE;
    queue->semaphore_get_slot--;        return FALSE;
  }

  //1- try to get a free slot
  while(slot_idx < TRANSMIT_QUEUE_LEN && queue->slot[slot_idx].status != ST_FREE)
    slot_idx++;

  if(slot_idx == TRANSMIT_QUEUE_LEN){
    //Transmit queue is full
    TRACE("\ttransmit_queue: check_free_space:  ANY SLOT AVAILABLE --> QUIT\n");DEBUG_PRINT_QUEUE;
    queue->semaphore_get_slot--;        return FALSE;
  }

  // slot available!
  // There is a free slot struct that could manage the transaction
  queue->slot[slot_idx].status =        ST_RESERVED;
  *idx =                                slot_idx;
  TRACE("\ttransmit_queue: check_free_space:  ASSIGNED SLOT %u\n", slot_idx);
  queue->semaphore_get_slot--;        return TRUE;
}

//Public
void transmit_queue_insert_slot(struct transmit_queue *queue, uint8_t idx, void *data, uint8_t priority){

  //0- verify user inputs
  #ifdef _TRANSMIT_QUEUE_PUBLIC_CHECKS_
  if(!transmit_queue_validate_slot_idx(queue, idx)) {
    TRACE("\transmit_queue: insert_slot:  POSSIBLE USAGE ERROR; SLOT %u WAS FOUNT NOT VALID. EXECUTION ABORTED\n", idx);
    return;
  }
  #endif // _TRANSMIT_QUEUE_PUBLIC_CHECKS_

  //1- verify data pointer
  #ifdef _TRANSMIT_QUEUE_PUBLIC_CHECKS_
  if(data == NULL){
    TRACE("\ttransmit_queue: insert_slot:  POSSIBLE USAGE ERROR: DATA POINTER OF SLOT %u IS 'NULL'. EXECUTION ABORTED\n", idx);
    return;
  }
  #endif // _TRANSMIT_QUEUE_PUBLIC_CHECKS_

  //2- set element pointer and priority
  queue->slot[idx].data = data;
  queue->slot[idx].priority = priority;

  //3- verify atomic operation
  if(++queue->semaphore_queue > 1){
    //We are interrupting another function which also attemps to modify the queue.
    //Save this action as pending and quit
    queue->pdg_action.action = ADD_QUEUE;
    queue->pdg_action.idx = idx;
    TRACE("\ttransmit_queue: insert_slot:  BUFFER BUSY. ACTION SAVED AS PENDING\n");
    queue->semaphore_queue--;   return;
  }

  //4- insert slot in queue
  transmit_queue_insert_slot_atomic(queue, idx);
  
  //5- execute pending action on queue
  transmit_queue_pending_action(queue);

  queue->semaphore_queue--;
}

//Public
bool_t transmit_queue_extract_slot(struct transmit_queue *queue, void **data){
  uint8_t queue_idx;

  //0- verify atomic operation
  if(++queue->semaphore_queue > 1) {
    //We are interrupting another function which also attemps to modify the queue.
    //this action cannot be saved as pending since it has to return an idx: so, just quit as action failed
    TRACE("\ttransmit_queue: extract_slot:  INTERRUPTING --> QUIT\n");
    queue->semaphore_queue--;   return FALSE;
  }
  
  //1- get first slot from queue
  queue_idx = queue->first_send;
  if(queue_idx == TRANSMIT_QUEUE_LEN){
    //Empty queue
    TRACE("\ttransmit_queue: extract_slot:  ANY SLOT IN QUEUE --> QUIT\n"); DEBUG_PRINT_QUEUE;
    queue->semaphore_queue--;   return FALSE;
  }
  #ifdef _TRANSMIT_QUEUE_PRIVATE_CHECKS_
  if(queue->slot[queue_idx].status != ST_READY){
    //If this happens there is an internal error in queue management. FIX IT!
    transmit_queue_init(queue);
    TRACE("\ttransmit_queue: extract_slot:  ERROR!!!! QUEUE CONTAINS A NON READY ELEMENT\n"); DEBUG_PRINT_QUEUE;
    return FALSE;
  }
  #endif // _TRANSMIT_QUEUE_PRIVATE_CHECKS_
  

  //2- extract first slot from queue
  queue->first_send =                   queue->slot[queue_idx].next_send;
  queue->slot[queue_idx].status =       ST_FREE;
  *data =                               queue->slot[queue_idx].data;
  TRACE("\ttransmit_queue: extract_slot:  SLOT %u EXTRACTED FROM QUEUE\n", queue_idx); DEBUG_PRINT_QUEUE;
  queue->semaphore_queue--;     return TRUE;
}

//Public
void transmit_queue_free_slot(struct transmit_queue *queue, uint8_t idx){

  //0- verify user inputs
  #ifdef _TRANSMIT_QUEUE_PUBLIC_CHECKS_
  if(!transmit_queue_validate_slot_idx(queue, idx)) {
    TRACE("\transmit_queue: free_slot:  POSSIBLE USAGE ERROR; SLOT %u WAS FOUNT NOT VALID. EXECUTION ABORTED\n", idx); DEBUG_PRINT_QUEUE;
    return;
  }
  #endif // _TRANSMIT_QUEUE_PUBLIC_CHECKS_

  //1- free slot
  queue->slot[idx].priority =   0;
  queue->slot[idx].next_send =  TRANSMIT_QUEUE_LEN;
  queue->slot[idx].status =     ST_FREE;
  TRACE("\ttransmit_queue: free_slot:  SLOT %u RELEASED\n", idx);
}

//Private
void transmit_queue_insert_slot_atomic(struct transmit_queue *queue, uint8_t idx){
  uint8_t slot_idx = queue->first_send;
  uint8_t slot_idx_previous = TRANSMIT_QUEUE_LEN;
  #ifdef _TRANSMIT_QUEUE_PRIVATE_CHECKS_
  uint8_t sec = 0; //security iteration counter (should be not necessary)
  #endif // _TRANSMIT_QUEUE_PRIVATE_CHECKS_

  //0- verify user inputs
  #ifdef _TRANSMIT_QUEUE_PRIVATE_CHECKS_
  if(!transmit_queue_validate_slot_idx(queue, idx)) {
    TRACE("\transmit_queue: insert_slot_atomic:  INTERNAL ERROR!!!! SLOT %u WAS FOUNT NOT VALID. EXECUTION ABORTED\n", idx); DEBUG_PRINT_QUEUE;
    return;
  }
  #endif // _TRANSMIT_QUEUE_PRIVATE_CHECKS_

  //1- insert slot in queue
  //1.1- Transmit queue is empty. Insert in first place
  if (slot_idx == TRANSMIT_QUEUE_LEN){
    queue->slot[idx].next_send =                TRANSMIT_QUEUE_LEN;
    queue->slot[idx].status =                   ST_READY;
    queue->first_send =                         idx;
    TRACE("\ttransmit_queue: insert_slot_atomic:  EMPTY QUEUE. SLOT %u INSERTED IN FIRST PLACE (PRIORITY = %u)\n", idx, queue->slot[idx].priority); DEBUG_PRINT_QUEUE;
    return;
  }

  //Follow queue...
  #ifdef _TRANSMIT_QUEUE_PRIVATE_CHECKS_
  while(queue->slot[slot_idx].status == ST_READY && queue->slot[slot_idx].priority >= queue->slot[idx].priority)
  #else
  while(queue->slot[slot_idx].priority >= queue->slot[idx].priority)
  #endif // _TRANSMIT_QUEUE_PRIVATE_CHECKS_
  {
    if(queue->slot[slot_idx].next_send < TRANSMIT_QUEUE_LEN){
      slot_idx_previous = slot_idx;
      slot_idx = queue->slot[slot_idx].next_send;
    }
    else{
      //1.2- End of queue. Insert at the end
      queue->slot[idx].next_send =              TRANSMIT_QUEUE_LEN;
      queue->slot[idx].status =                 ST_READY;
      queue->slot[slot_idx].next_send =         idx;
      TRACE("\ttransmit_queue: insert_slot_atomic:  SLOT %u INSERTED AT THE END (PRIORITY = %u)\n", idx, queue->slot[idx].priority); DEBUG_PRINT_QUEUE;
      return;
    }
    #ifdef _TRANSMIT_QUEUE_PRIVATE_CHECKS_
    if(++sec == TRANSMIT_QUEUE_LEN){
      //Security counter. This should never happen!!!
      //If this happens there is an internal error in queue management. FIX IT!
      transmit_queue_init(queue);
      TRACE("\ttransmit_queue: insert_slot_atomic:  ERROR!!!! QUEUE MAKES A LOOP (INFINITE QUEUE)\n"); DEBUG_PRINT_QUEUE;
      return;
    }
    #endif // _TRANSMIT_QUEUE_PRIVATE_CHECKS_
  }
  #ifdef _TRANSMIT_QUEUE_PRIVATE_CHECKS_
  if(queue->slot[slot_idx].status != ST_READY){
    //If this happens there is an internal error in queue management. FIX IT!
    transmit_queue_init(queue);
    TRACE("\ttransmit_queue: insert_slot_atomic:  ERROR!!!! QUEUE CONTAINS A NON READY ELEMENT\n"); DEBUG_PRINT_QUEUE;
    return;
  }
  #endif // _TRANSMIT_QUEUE_PRIVATE_CHECKS_
  
  //1.3- Insertion point found while following the queue. Insert here
  queue->slot[idx].next_send =                  slot_idx;
  queue->slot[idx].status =                     ST_READY;
  if(slot_idx_previous < TRANSMIT_QUEUE_LEN) {
    queue->slot[slot_idx_previous].next_send = idx;
    TRACE("\ttransmit_queue: insert_slot_atomic:  SLOT %u INSERTED IN THE MIDDLE (PRIORITY = %u)\n", idx, queue->slot[idx].priority); DEBUG_PRINT_QUEUE;
  }
  else {
    queue->first_send =                         idx;
    TRACE("\ttransmit_queue: insert_slot_atomic:  SLOT %u INSERTED IN FIRST PLACE (PRIORITY = %u)\n", idx, queue->slot[idx].priority); DEBUG_PRINT_QUEUE;
  }
}

//Private
void transmit_queue_pending_action(struct transmit_queue *queue){
  switch(queue->pdg_action.action){
    case ADD_QUEUE:
      TRACE("\ttransmit_queue: pending_action: ADD slot %u in QUEUE\n", queue->pdg_action.idx);
      transmit_queue_insert_slot_atomic(queue, queue->pdg_action.idx);
      break;
    default:
      break;
  }
  queue->pdg_action.action = NONE;
}

#if defined (_TRANSMIT_QUEUE_PUBLIC_CHECKS_) || defined (_TRANSMIT_QUEUE_PRIVATE_CHECKS_)
//Private
bool_t transmit_queue_validate_slot_idx(struct transmit_queue *queue, uint8_t idx){
//this function verifies that the slot identified by 'idx':
// 0 - exists
// 1 - is being used
 
  //0- verify slot index
  if(idx >= TRANSMIT_QUEUE_LEN)
  {     TRACE("\ttransmit_queue: validate_slot_idx:  SLOT %u DOESN'T EXIST\n", idx);    return FALSE; }

  //1- verify slot status
  if(queue->slot[idx].status != ST_RESERVED)
  {     TRACE("\ttransmit_queue: validate_slot_idx:  SLOT %u IS NOT IN 'ST_RESERVED' STATUS (status = %u)\n", idx, queue->slot[idx].status); return FALSE; }

  return TRUE;
}
#endif
