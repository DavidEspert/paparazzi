/*
 * Copyright (C) 2013 Guillem Jornet i Nasarre
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef _TRANSMIT_QUEUE_H_
#define _TRANSMIT_QUEUE_H_

/**
 * @file mcu_periph/transmit_queue.h
 *
 * This file provides support for data management according to assigned priorities.
 * Data pointer is stored in a 'slot' of the 'transmit_queue'. Once a slot is ready,
 * it will be inserted in the transmition queue chain.
 * (This file has originally been developed for output device's transactions)
 * 
 * The normal use should be:
 * 0) init:             initializes a 'transmit_queue'.
 * ... and for any element we want to add...
 * 1) check_free_space: if a slot is available, 'slot_idx' is set, slot is marked as RESERVED
 *                      and the function returns true.
 * 2) insert_slot:      if the slot is RESERVED (otherwise this function does nothing),
 *                      data pointer is set and the slot is inserted in transmition queue
 *                      (according to its priority) and marked as READY.
 * 3) extract_slot:     if there is/are slot/s in queue, first slot is extracted, data pointer is set,
 *                      slot is marked as FREE and the function returns true.
 * 4) free_slot:        if slot is RESERVED it will be marked as FREE.
 * 
 * NOTE: Actions 1, 2 and 3 must be executed atomically. In case of 'insert_slot' (2), if it cannot be
 *       executed at the moment but later on, it will be saved as 'pending action'.
 */

#include <stdint.h>
#include "std.h"        //required for bool_t definition

//Slot status
  //enum queue_status {ST_FREE, ST_RESERVED, ST_READY}
#define ST_FREE 1
#define ST_RESERVED 2
#define ST_READY 3

//num of slots available (1 slot --> 1 transmit element)
#ifndef TRANSMIT_QUEUE_LEN //you can define your own TRANSMIT_QUEUE_LEN
#define TRANSMIT_QUEUE_LEN      16
#endif

//identifiers for pending actions
#define NONE 0
#define ADD_QUEUE 1

//Declaration of initialized queue: i.e. 'struct transmit_queue queue = INITIALIZED_TRANSMIT_QUEUE;'
#define INITIALIZED_TRANSMIT_QUEUE { \
  .slot[0 ... (TRANSMIT_QUEUE_LEN-1)] = { .status = ST_FREE, .data = NULL, .priority = 0, .next_send = TRANSMIT_QUEUE_LEN}, \
  .first_send = TRANSMIT_QUEUE_LEN, \
  .semaphore_get_slot = 0, \
  .semaphore_queue = 0, \
  .pdg_action.action = NONE \
}

//Some actions have to be executed atomically.
//If an action cannot be executed and has not to return a parameter, it will be stored and executed when possible
struct tx_queue_pending_action {
  uint8_t       action;
  uint8_t       idx;
};

struct transmit_slot{
  uint8_t       status;         // slot status {ST_FREE, ST_RESERVED, ST_READY}
  void *        data;           // pointer to transaction data
  uint8_t       priority;       // transaction priority
  uint8_t       next_send;      // index of next slot to be sent
};

struct transmit_queue{
  struct transmit_slot  slot[TRANSMIT_QUEUE_LEN];
  uint8_t               first_send;
  uint8_t               semaphore_get_slot; //This is not be necessary with sequential code
  uint8_t               semaphore_queue;
  struct tx_queue_pending_action pdg_action;
};

// FUNCTIONS -----------------------------------------------------
//Initialize transmit queue.
void transmit_queue_init(struct transmit_queue *queue);

//Get a free slot.
//      If a slot is available (ST_FREE), it will be marked as ST_RESERVED
//      and 'idx' will be set with its index.
bool_t transmit_queue_check_free_space(struct transmit_queue *queue, uint8_t *idx);

//Insert a slot in transmit queue.
//      If slot is ST_RESERVED, it will be marked as ST_READY and insertion
//      will be done according to the assigned priority.
void transmit_queue_insert_slot(struct transmit_queue *queue, uint8_t idx, void *data, uint8_t priority);

//Get first slot from queue.
//      If a slot is available in queue (ST_READY), it will be marked as ST_FREE
//      and 'data' will be set with transaction data pointer value.
bool_t transmit_queue_extract_slot(struct transmit_queue *queue, void **data);

//Set a slot as free.
//      If slot is ST_RESERVED, it will be marked as ST_FREE.
void transmit_queue_free_slot(struct transmit_queue *queue, uint8_t idx);

#endif // _TRANSMIT_QUEUE_H_