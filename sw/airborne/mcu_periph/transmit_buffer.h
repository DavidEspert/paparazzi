#ifndef _TRANSMIT_BUFFER_H_
#define _TRANSMIT_BUFFER_H_

#include <stdint.h>
#include <string.h> //required for memcpy

//Buffer status
  //enum tx_buff_status {ST_FREE, ST_RESERVED, ST_READY, ST_SENDING}
#define ST_FREE 1
#define ST_RESERVED 2
#define ST_READY 3
#define ST_SENDING 4

#define TX_BUFF_TRUE 1
#define TX_BUFF_FALSE 0

//num of slots available (1 slot --> 1 message)
#define TX_BUFF_NUM_SLOTS	10
//max storable data (adjusted for a total 'transmit_slot' size of 256)
//#define MAX_DATA_LENGTH		253
#define MAX_DATA_LENGTH		32

//identifiers for pending actions
#define NONE 0
#define ADD_QUEUE 1
#define	RMV_QUEUE 2

//Declaration of initialized buffer: i.e. 'struct transmit_buffer tx_buff = INITIALIZED_TX_BUFFER;'
#define INITIALIZED_TX_BUFFER { \
    .slot[0 ... (TX_BUFF_NUM_SLOTS-1)] = { .length = 0, .status = ST_FREE, .priority = 0, .next_slot = TX_BUFF_NUM_SLOTS}, \
    .first_output = TX_BUFF_NUM_SLOTS, \
    .semaphore_get = 0, \
    .semaphore_queue = 0, \
    .pdg_action.action = NONE \
}

struct pending_action {
  uint8_t	action;
  uint8_t	idx;
};

struct transmit_slot {
  uint8_t	buffer[MAX_DATA_LENGTH];
  uint8_t	length;
  uint8_t	status;
  uint8_t	priority;
  uint8_t	next_slot;
//  transmit_slot *next_buff;  (use next_slot instead)
};

//#pragma DATA_ALIGN(256)
struct transmit_buffer {
  struct transmit_slot	slot[TX_BUFF_NUM_SLOTS];
  uint8_t		first_output; //first slot to be send/sending
  uint8_t		semaphore_get;
  uint8_t		semaphore_queue;
  struct pending_action pdg_action;
};


void tx_buffer_init(struct transmit_buffer *tx_buff);
uint8_t tx_buffer_get_slot(struct transmit_buffer *tx_buff, uint8_t length, uint8_t *idx);
uint8_t * tx_buffer_get_slot_pointer(struct transmit_buffer *tx_buff, uint8_t idx);
//void fill_buffer(struct transmit_buffer *tx_buff, uint8_t idx, uint8_t offset, uint8_t *origin, uint8_t length);
void try_insert_slot_in_queue(struct transmit_buffer *tx_buff, uint8_t idx, uint8_t priority);
void try_extract_slot_from_queue(struct transmit_buffer *tx_buff, uint8_t idx);
void tx_buffer_free_slot(struct transmit_buffer *tx_buff, uint8_t idx);

#endif // _TRANSMIT_BUFFER_H_