#include "dynamic_buffer.h"
#include <string.h>     //required for memcpy


// NOTE: About '_DYNAMIC_BUFFER_PUBLIC_CHECKS_' and '_DYNAMIC_BUFFER_PRIVATE_CHECKS_'
// Checks ensure that a given parameter 'slot index' or 'initial memory pointer'
// corresponds to an existing and correctly initialized mem space.
// PUBLIC  checks find possible user errors --> What to do with this? Always enabled? Just for Debug?
// PRIVATE checks find internal errors --> DISABLE AFTER DEVELOPMENT

#ifdef _DYNAMIC_BUFFER_TRACES_
#include <stdio.h>
#define TRACE(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);
#define DEBUG_PRINT_BUFFER_MEM { \
  uint8_t i = buff->first_mem; \
  TRACE("\tBuffer MEM  order (slot):    \t{"); \
  while( i < TX_BUFF_NUM_SLOTS){ \
    TRACE("%3u, ", i); \
    i = buff->slot[i].next_mem; \
  } \
  TRACE("}\n"); \
\
  i = buff->first_mem; \
  TRACE("\tBuffer MEM  order (init):    \t{"); \
  while( i < TX_BUFF_NUM_SLOTS){ \
    TRACE("%3u, ",buff->slot[i].init); \
    i = buff->slot[i].next_mem; \
  } \
  TRACE("}\n"); \
\
  i = buff->first_mem; \
  TRACE("\tBuffer MEM  order (end):     \t{"); \
  while( i < TX_BUFF_NUM_SLOTS){ \
    TRACE("%3u, ",(buff->slot[i].init + buff->slot[i].length -1)); \
    i = buff->slot[i].next_mem; \
  } \
  TRACE("}\n"); \
}

#else
#define TRACE(...)
#define DEBUG_PRINT_BUFFER_MEM {}
#endif //_DYNAMIC_BUFFER_TRACES_




//MEM_HEADER_LEN is the length of a header added to every assigned memory space.
//mem_header contains the 'slot_idx' that manages that mem space
#define MEM_HEADER_LEN  1
/*struct mem_header {
  uint8_t slot_idx;
};
#define MEM_HEADER_LEN  sizeof(struct mem_header)*/

//Private functions
void dynamic_buffer_free_slot_atomic(struct dynamic_buffer *buff, uint8_t idx);
void dynamic_buffer_pending_action(struct dynamic_buffer *buff);
#if defined (_DYNAMIC_BUFFER_PUBLIC_CHECKS_) || defined (_DYNAMIC_BUFFER_PRIVATE_CHECKS_)
bool_t dynamic_buffer_validate_slot_idx(struct dynamic_buffer *buff, uint8_t idx);
bool_t dynamic_buffer_validate_slot_pointer(struct dynamic_buffer *buff, uint8_t *p);
#endif


struct dynamic_buffer dynamic_buff  = INITIALIZED_DYNAMIC_BUFFER;


//Public
void dynamic_buffer_init(struct dynamic_buffer *buff) {
  for (uint8_t idx = 0; idx < TX_BUFF_NUM_SLOTS; idx++) {
    buff->slot[idx].status =            ST_FREE;
    buff->slot[idx].init =              0;
    buff->slot[idx].length =            0;
    buff->slot[idx].next_mem =          TX_BUFF_NUM_SLOTS;
  }
  buff->first_mem =                     TX_BUFF_NUM_SLOTS;
  buff->semaphore_get_mem =             0;
  buff->pdg_action.action =             NONE;
}

//Public
bool_t dynamic_buffer_check_free_space(struct dynamic_buffer *buff, uint16_t data_length, uint8_t *idx){
  uint16_t total_length;
  uint8_t slot_idx = 0;
  uint8_t slot_idx2;
  #ifdef _DYNAMIC_BUFFER_PRIVATE_CHECKS_
  uint8_t sec = 0; //security iteration counter (should be not necessary)
  #endif // _DYNAMIC_BUFFER_PRIVATE_CHECKS_
  struct T_candidate {
    uint16_t  init;
    uint16_t  length;
    uint8_t  prev_mem;
    uint8_t  next_mem;
  } candidate = { BUFFER_LENGTH, BUFFER_LENGTH, TX_BUFF_NUM_SLOTS, TX_BUFF_NUM_SLOTS};

  //0- verify atomic operation
  if(++buff->semaphore_get_mem > 1) {
    //We are interrupting another 'dynamic_buffer_check_free_space' call
    TRACE("\tdynamic_buffer: get_slot:  INTERRUPTING --> QUIT\n"); DEBUG_PRINT_BUFFER_MEM;
    buff->semaphore_get_mem--;
    return FALSE;
  }

  //1- verify data length
  if(data_length == 0){ 
    buff->semaphore_get_mem--;  return FALSE;
  }
  if(data_length > (BUFFER_LENGTH - MEM_HEADER_LEN)){ 
    //(this step is not necessary but it saves time in case of 'data_length > (BUFFER_LENGTH - MEM_HEADER_LEN)'
    TRACE("\tdynamic_buffer: get_slot:  LENGTH EXCEEDED --> QUIT\n"); DEBUG_PRINT_BUFFER_MEM;
    buff->semaphore_get_mem--;  return FALSE;
  }
  total_length = data_length + MEM_HEADER_LEN;

  //2- try to get a slot
  while(slot_idx < TX_BUFF_NUM_SLOTS && buff->slot[slot_idx].status != ST_FREE)
    slot_idx++;

  if(slot_idx == TX_BUFF_NUM_SLOTS){
    //Buffer is full
    TRACE("\tdynamic_buffer: get_slot:  ANY SLOT AVAILABLE --> QUIT\n"); DEBUG_PRINT_BUFFER_MEM;
    buff->semaphore_get_mem--;       return FALSE;
  }

  // slot available!
  // That is: there is a free slot struct that could manage the required memory space but it
  // must be checked if there is memory enough

  //3- look for memory space
  slot_idx2 = buff->first_mem;
  //3.1- There is any slot in memory
  if(slot_idx2 == TX_BUFF_NUM_SLOTS) {
    candidate.init =            0;
    candidate.length =          BUFFER_LENGTH;
    candidate.prev_mem =        TX_BUFF_NUM_SLOTS;
    candidate.next_mem =        TX_BUFF_NUM_SLOTS;
    //TRACE("\tdynamic_buffer: get_slot:  MEMORY FOUND : init = %u, length available = %u, previous slot = %u, next slot = %u\n", candidate.init, candidate.length, candidate.prev_mem, candidate.next_mem);
    goto end;
  }
  //3.2- There is a hole at the beginning
  if(buff->slot[slot_idx2].init >= total_length) {
    candidate.init =            0;
    candidate.length =          buff->slot[slot_idx2].init;
    candidate.prev_mem =        TX_BUFF_NUM_SLOTS;
    candidate.next_mem =        slot_idx2;// = buff->first_mem;
    //TRACE("\tdynamic_buffer: get_slot:  MEMORY FOUND : init = %u, length available = %u, previous slot = %u, next slot = %u\n", candidate.init, candidate.length, candidate.prev_mem, candidate.next_mem);
  }
  //3.3- Check holes after the existing slots
  while (slot_idx2 < TX_BUFF_NUM_SLOTS) {
    uint8_t  next_mem =  buff->slot[slot_idx2].next_mem;
    uint16_t hole_init = buff->slot[slot_idx2].init + buff->slot[slot_idx2].length;
    uint16_t hole_length;
    if(next_mem < TX_BUFF_NUM_SLOTS)    hole_length = buff->slot[next_mem].init - hole_init;
    else                                hole_length = BUFFER_LENGTH - hole_init;
    //look for the smallest suitable hole in memory. Leave larger ones for larger messages
    #ifdef _DYNAMIC_BUFFER_TRACES_
    if (hole_length >= total_length){
      //TRACE("\tdynamic_buffer: get_slot:  MEMORY FOUND : init = %u, length available = %u, previous slot = %u, next slot = %u\n", hole_init, hole_length, slot_idx2, next_mem);
    }
    #endif // _DYNAMIC_BUFFER_TRACES_
    if (hole_length >= total_length && hole_length < candidate.length){
      candidate.init =        hole_init;
      candidate.length =      hole_length;
      candidate.prev_mem =    slot_idx2;
      candidate.next_mem =    next_mem;
    }
    slot_idx2 = next_mem;

    #ifdef _DYNAMIC_BUFFER_PRIVATE_CHECKS_
    if(++sec == TX_BUFF_NUM_SLOTS){
      //Security counter. This should never happen!!!
      //If this happens there is an internal error in buffer management. FIX IT!
      dynamic_buffer_init(buff);
      TRACE("\tdynamic_buffer: get_slot:  INTERNAL ERROR!!!! 'next_mem' MAKES AN INFINITE LOOP\n"); DEBUG_PRINT_BUFFER_MEM;
      return FALSE;
    }
    #endif // _DYNAMIC_BUFFER_PRIVATE_CHECKS_
  }

end:
  if(candidate.init < BUFFER_LENGTH) {
    buff->slot[slot_idx].status =    ST_RESERVED;
    buff->slot[slot_idx].init =      candidate.init;
    buff->slot[slot_idx].length =    total_length;
    if(candidate.prev_mem < TX_BUFF_NUM_SLOTS)
      buff->slot[candidate.prev_mem].next_mem = slot_idx;
    else
      buff->first_mem =              slot_idx;
    buff->slot[slot_idx].next_mem =  candidate.next_mem;

    buff->buffer[candidate.init] =   slot_idx;
    *idx =                           slot_idx;
    TRACE("\tdynamic_buffer: get_slot:  ASSIGNED SLOT %u (BYTES REQUIRED = %u DATA + %u HEADER)\n", slot_idx, data_length, MEM_HEADER_LEN); DEBUG_PRINT_BUFFER_MEM;
    buff->semaphore_get_mem--;	return TRUE;
  }
  else {
    TRACE("\tdynamic_buffer: get_slot:  NO MEMORY AVAILABLE (BYTES REQUIRED = %u DATA + %u HEADER) --> QUIT\n", data_length, MEM_HEADER_LEN); DEBUG_PRINT_BUFFER_MEM;
    buff->semaphore_get_mem--;	return FALSE;
  }
}

//Public
uint8_t * dynamic_buffer_get_slot_pointer(struct dynamic_buffer *buff, uint8_t idx){
  #ifdef _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  if(!dynamic_buffer_validate_slot_idx(buff, idx)) return NULL;
  #endif // _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  return &(buff->buffer[(buff->slot[idx].init + MEM_HEADER_LEN)]);
}

//Public
uint8_t dynamic_buffer_get_slot_index(__attribute__((unused)) struct dynamic_buffer *buff, uint8_t* ptr){
  #ifdef _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  if(!dynamic_buffer_validate_slot_pointer(buff, ptr)) return TX_BUFF_NUM_SLOTS;
  #endif // _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  return *(ptr - 1);
}

//Public
uint8_t dynamic_buffer_read_a_byte(struct dynamic_buffer *buff, uint8_t idx, uint8_t byte_idx){
//What to do in case of failure? Should this function require as input a pointer to the byte and return TRUE/FALSE?

  //0- verify user inputs
  #ifdef _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  if(!dynamic_buffer_validate_slot_idx(buff, idx)) {
    TRACE("\tdynamic_buffer: read_a_byte:  POSSIBLE USAGE ERROR; SLOT %u WAS FOUNT NOT VALID. EXECUTION ABORTED\n", idx);
    return 0;
  }
  #endif // _DYNAMIC_BUFFER_PUBLIC_CHECKS_

  //1- verify data length
  #ifdef _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  if( (MEM_HEADER_LEN + byte_idx) > buff->slot[idx].length){
    TRACE("\tdynamic_buffer: read_a_byte:  POSSIBLE USAGE ERROR ATTEMPTED TO READ OUT OF THE ASSIGNED MEM SPACE FOR SLOT %u. EXECUTION ABORTED\n", idx);
    return 0;
  }
  #endif // _DYNAMIC_BUFFER_PUBLIC_CHECKS_

  return (buff->buffer[(buff->slot[idx].init + MEM_HEADER_LEN + byte_idx)]);
}

//Public
bool_t dynamic_buffer_read(struct dynamic_buffer *buff, uint8_t idx, uint16_t offset, uint8_t *destiny, uint16_t length){

  //0- verify user inputs
  #ifdef _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  if(!dynamic_buffer_validate_slot_idx(buff, idx)) {
    TRACE("\tdynamic_buffer: fill:  POSSIBLE USAGE ERROR; SLOT %u WAS FOUNT NOT VALID. EXECUTION ABORTED\n", idx);
    return FALSE;
  }
  #endif // _DYNAMIC_BUFFER_PUBLIC_CHECKS_

  //1- verify data length
  #ifdef _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  if( (MEM_HEADER_LEN + offset + length) > buff->slot[idx].length){
    TRACE("\tdynamic_buffer: fill:  POSSIBLE USAGE ERROR ATTEMPTED TO READ OUT OF THE ASSIGNED MEM SPACE FOR SLOT %u. EXECUTION ABORTED\n", idx);
    return FALSE;
  }
  #endif // _DYNAMIC_BUFFER_PUBLIC_CHECKS_

  memcpy(destiny, &(buff->buffer[(buff->slot[idx].init + MEM_HEADER_LEN + offset)]), length);
  return TRUE;
}
  
//Public
bool_t dynamic_buffer_write(struct dynamic_buffer *buff, uint8_t idx, uint16_t offset, uint8_t *origin, uint16_t length){

  //0- verify user inputs
  #ifdef _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  if(!dynamic_buffer_validate_slot_idx(buff, idx)) {
    TRACE("\tdynamic_buffer: fill:  POSSIBLE USAGE ERROR; SLOT %u WAS FOUNT NOT VALID. EXECUTION ABORTED\n", idx);
    return FALSE;
  }
  #endif // _DYNAMIC_BUFFER_PUBLIC_CHECKS_

  //1- verify data length
  #ifdef _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  if( (MEM_HEADER_LEN + offset + length) > buff->slot[idx].length){
    TRACE("\tdynamic_buffer: fill:  POSSIBLE USAGE ERROR ATTEMPTED TO WRITE OUT OF THE ASSIGNED MEM SPACE FOR SLOT %u. EXECUTION ABORTED\n", idx);
    return FALSE;
  }
  #endif // _DYNAMIC_BUFFER_PUBLIC_CHECKS_

  memcpy(&(buff->buffer[(buff->slot[idx].init + MEM_HEADER_LEN + offset)]), origin, length);
  return TRUE;
}
  
//Public
void dynamic_buffer_free_slot_idx(struct dynamic_buffer *buff, uint8_t idx){

  //0- verify user inputs
  #ifdef _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  if(!dynamic_buffer_validate_slot_idx(buff, idx)) {
    TRACE("\tdynamic_buffer: free_slot_idx:  POSSIBLE USAGE ERROR; SLOT %u WAS FOUNT NOT VALID. EXECUTION ABORTED\n", idx); DEBUG_PRINT_BUFFER_MEM;
    return;
  }
  #endif // _DYNAMIC_BUFFER_PUBLIC_CHECKS_

  //1- verify atomic operation
  if(++buff->semaphore_get_mem > 1){
    //We are interrupting another function which also attemps to modify the dynamic_buffer.
    //Save this action as pending and quit
    buff->pdg_action.action =  FREE_SLOT;
    buff->pdg_action.idx =     idx;
    TRACE("\tdynamic_buffer: free_slot_idx:  BUFFER BUSY. ACTION SAVED AS PENDING\n");
    buff->semaphore_get_mem--;    return;
  }

  //2- free slot
  dynamic_buffer_free_slot_atomic(buff, idx);
  
  //3- execute pending action
  dynamic_buffer_pending_action(buff);

  buff->semaphore_get_mem--;
}

//Public
void dynamic_buffer_free_slot_pointer(struct dynamic_buffer *buff, uint8_t *ptr){
  uint8_t idx;

  //0- verify user inputs
  #ifdef _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  if(!dynamic_buffer_validate_slot_pointer(buff, ptr)) {
    TRACE("\tdynamic_buffer: free_slot_pointer:  POSSIBLE USAGE ERROR; POINTER %p WAS FOUNT NOT VALID. EXECUTION ABORTED\n", ptr); DEBUG_PRINT_BUFFER_MEM;
    return;
  }
  #endif // _DYNAMIC_BUFFER_PUBLIC_CHECKS_
  
  //1- get slot index
  idx = dynamic_buffer_get_slot_index(buff, ptr);

  //2- verify atomic operation
  if(++buff->semaphore_get_mem > 1){
    //We are interrupting another function which also attemps to modify the dynamic_buffer.
    //Save this action as pending and quit
    buff->pdg_action.action =  FREE_SLOT;
    buff->pdg_action.idx =     idx;
    TRACE("\tdynamic_buffer: free_slot_pointer:  BUFFER BUSY. ACTION SAVED AS PENDING\n");
    buff->semaphore_get_mem--;    return;
  }

  //3- free slot
  dynamic_buffer_free_slot_atomic(buff, idx);
  
  //4- execute pending action
  dynamic_buffer_pending_action(buff);

  buff->semaphore_get_mem--;
}

//Private
void dynamic_buffer_free_slot_atomic(struct dynamic_buffer *buff, uint8_t idx){
  uint8_t slot_idx;
  #ifdef _DYNAMIC_BUFFER_PRIVATE_CHECKS_
  uint8_t sec = 0; //security iteration counter (should be not necessary)
  #endif // _DYNAMIC_BUFFER_PRIVATE_CHECKS_

  //0- verify inputs
  #ifdef _DYNAMIC_BUFFER_PRIVATE_CHECKS_
  //this is not a user function. If any error happens it is internal. FIX IT!
  if(!dynamic_buffer_validate_slot_idx(buff, idx)) {
    TRACE("\tdynamic_buffer: free_slot_atomic:  INTERNAL ERROR!!!! SLOT %u WAS FOUNT NOT VALID. EXECUTION ABORTED\n", idx); DEBUG_PRINT_BUFFER_MEM;
    return;
  }
  #endif // _DYNAMIC_BUFFER_PRIVATE_CHECKS_

  //1- find slot in memory
  slot_idx = buff->first_mem;

  //case A- memory is empty
  if (slot_idx >= TX_BUFF_NUM_SLOTS){
    goto end;
  }
  //case B- slot is the first element in memory
  if(slot_idx == idx){
    buff->first_mem = buff->slot[idx].next_mem;
    goto end;
  }
  //case C- slot is in the middle
  while(buff->slot[slot_idx].next_mem < TX_BUFF_NUM_SLOTS){
    if(buff->slot[slot_idx].next_mem == idx){
      buff->slot[slot_idx].next_mem = buff->slot[idx].next_mem;
      goto end;
    }
    slot_idx = buff->slot[slot_idx].next_mem;

    #ifdef _DYNAMIC_BUFFER_PRIVATE_CHECKS_
    if(++sec == TX_BUFF_NUM_SLOTS){
      //Security counter. This should never happen!!!
      //If this happens there is an internal error in buffer management. FIX IT!
      dynamic_buffer_init(buff);
      TRACE("\tdynamic_buffer: free_slot_atomic:  INTERNAL ERROR!!!! MEMORY SEARCH MAKES AN INFINITE LOOP.  -- BUFFER RESET HAS BEEN DONE --\n"); DEBUG_PRINT_BUFFER_MEM;
      return;
    }
    #endif // _DYNAMIC_BUFFER_PRIVATE_CHECKS_
  }
  //case D- slot is already free
end:
/*  buff->slot[idx].init =                0;
  buff->slot[idx].length =              0;
  buff->slot[idx].next_mem =            TX_BUFF_NUM_SLOTS;*/
  buff->slot[idx].status =              ST_FREE;
  TRACE("\tdynamic_buffer: free_slot_atomic:  SLOT %u EXTRACTED FROM MEMORY AND RELEASED\n", idx); DEBUG_PRINT_BUFFER_MEM;
}

//Private
void dynamic_buffer_pending_action(struct dynamic_buffer *buff){
  switch(buff->pdg_action.action){
    case FREE_SLOT:
      TRACE("\tdynamic_buffer: pending_action: FREE slot %u\n", buff->pdg_action.idx);
      dynamic_buffer_free_slot_atomic(buff, buff->pdg_action.idx);
      break;
    default:
      break;
  }
  buff->pdg_action.action = NONE;
}

#if defined (_DYNAMIC_BUFFER_PUBLIC_CHECKS_) || defined (_DYNAMIC_BUFFER_PRIVATE_CHECKS_)
//Private
bool_t dynamic_buffer_validate_slot_idx(struct dynamic_buffer *buff, uint8_t idx){
//this function verifies that the slot identified by 'idx':
// 0 - exists
// 1 - is being used
// 2 - mem space is correctly initialized
 
  //0- verify slot index
  if(idx >= TX_BUFF_NUM_SLOTS)
  {     TRACE("\tdynamic_buffer: validate_slot_idx:  SLOT %u DOESN'T EXIST\n", idx);    return FALSE; }

  //1- verify slot status
  if(buff->slot[idx].status != ST_RESERVED)
  {     TRACE("\tdynamic_buffer: validate_slot_idx:  SLOT %u IS NOT IN 'ST_RESERVED' STATUS\n", idx); return FALSE; }

  //2- verify memory initialization (check mem_header)
  if(buff->buffer[buff->slot[idx].init] != idx)
  {     TRACE("\tdynamic_buffer: validate_slot_idx:  SLOT %u IS NOT CORRECTLY INITIALIZED\n", idx);   return FALSE; }

  return TRUE;
}

//Private
bool_t dynamic_buffer_validate_slot_pointer(struct dynamic_buffer *buff, uint8_t *ptr){
  uint8_t idx = *(ptr - 1);
  uint8_t *slot_ptr;

  //0- verify slot index
  if(idx >= TX_BUFF_NUM_SLOTS)
  {     TRACE("\tdynamic_buffer: validate_slot_pointer:  SLOT %u DOESN'T EXIST\n", idx);    return FALSE; }

  //0- verify slot pointer
  slot_ptr = dynamic_buffer_get_slot_pointer(buff, idx);
  if(slot_ptr != ptr)
  {     TRACE("\tdynamic_buffer: validate_slot_pointer:  POINTER %p DOESN'T CORRESPOND TO AN INITIAL MEMORY SPACE\n", ptr);    return FALSE; }

  return TRUE;
}
#endif
