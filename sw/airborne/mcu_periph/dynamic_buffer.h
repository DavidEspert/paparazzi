#ifndef _DYNAMIC_BUFFER_H_
#define _DYNAMIC_BUFFER_H_

#include <stdint.h>
#include "std.h"	//required for bool_t definition

//Buffer status
  //enum buff_status {ST_FREE, ST_RESERVED}
#define ST_FREE 1
#define ST_RESERVED 2

//num of slots available
#define TX_BUFF_NUM_SLOTS       16
#define BUFFER_LENGTH           512

//identifiers for pending actions
#define NONE 0
#define	FREE_SLOT 1


struct dyn_buff_pending_action {
  uint8_t       action;
  uint8_t       idx;
};
#define INITIALIZED_DYNAMIC_BUFFER_PDG_ACTION { .action = NONE, .idx = TX_BUFF_NUM_SLOTS}

struct buffer_slot {
  uint8_t       status;         // slot status {FREE, RESERVED}
  uint16_t      init;           // position of first slot byte in buffer
  uint16_t      length;         // slot length
  uint8_t       next_mem;       // index of next slot in memory
};
#define INITIALIZED_DYNAMIC_BUFFER_SLOT { .status = ST_FREE, .init = 0, .length = 0, .next_mem = TX_BUFF_NUM_SLOTS}

//#pragma DATA_ALIGN(256)
struct dynamic_buffer {
  uint8_t               buffer[BUFFER_LENGTH];
  struct buffer_slot    slot[TX_BUFF_NUM_SLOTS];
  uint8_t               first_mem;      //first slot in memory
  uint8_t               semaphore_get_mem;
  struct dyn_buff_pending_action pdg_action;
};
//Declaration of initialized buffer: i.e. 'struct dynamic_buffer dynamic_buff = INITIALIZED_DYNAMIC_BUFFER;'
#define INITIALIZED_DYNAMIC_BUFFER { \
  .slot[0 ... (TX_BUFF_NUM_SLOTS-1)] = INITIALIZED_DYNAMIC_BUFFER_SLOT, \
  .first_mem = TX_BUFF_NUM_SLOTS, \
  .semaphore_get_mem = 0, \
  .pdg_action = INITIALIZED_DYNAMIC_BUFFER_PDG_ACTION \
}
extern struct dynamic_buffer dynamic_buff;



// FUNCTIONS -----------------------------------------------------
//Initialize dynamic buffer.
void dynamic_buffer_init(struct dynamic_buffer *buff);

//Get a free slot.
//      If a slot is available (ST_FREE), it will be marked as ST_RESERVED,
//      'idx' will be set with slot index and function will return TRUE.
bool_t dynamic_buffer_check_free_space(struct dynamic_buffer *buff, uint16_t length, uint8_t *idx);

//Get slot pointer.
//      Returns origin address of the reserved memory slot.
//      Returns NULL in case of failure.
uint8_t * dynamic_buffer_get_slot_pointer(struct dynamic_buffer *buff, uint8_t idx);

//Get slot index.
//      Returns slot index from its initial memory address.
//      Returns TX_BUFF_NUM_SLOTS in case of failure.
uint8_t dynamic_buffer_get_slot_index(struct dynamic_buffer *buff, uint8_t* ptr);

//Get a byte.
//      Returns the byte indicated by 'byte_idx' from the specified slot.
uint8_t dynamic_buffer_read_a_byte(struct dynamic_buffer *buff, uint8_t idx, uint8_t byte_idx);

//Read slot
//      Write 'length' bytes, from the ST_RESERVED slot (with an offset) to 'destiny' address.
bool_t dynamic_buffer_read(struct dynamic_buffer *buff, uint8_t idx, uint16_t offset, uint8_t *destiny, uint16_t length);

//Write slot.
//      Write 'length' bytes, from 'origin' address, in the ST_RESERVED slot (with an offset).
bool_t dynamic_buffer_write(struct dynamic_buffer *buff, uint8_t idx, uint16_t offset, uint8_t *origin, uint16_t length);

//Set a slot as free from its index 'idx'
//      If slot corresponding to 'idx' is ST_RESERVED, it will be marked as ST_FREE.
void dynamic_buffer_free_slot_idx(struct dynamic_buffer *buff, uint8_t idx);

//Set a slot as free from its initial address 'ptr'
//      If slot corresponding to 'ptr' is ST_RESERVED, it will be marked as ST_FREE.
void dynamic_buffer_free_slot_pointer(struct dynamic_buffer *buff, uint8_t *ptr);

#endif // _DYNAMIC_BUFFER_H_