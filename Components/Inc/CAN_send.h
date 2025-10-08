#ifndef _CAN_SEND_H
#define _CAN_SEND_H

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "string.h"

#define CAN_QUEUE_LENGTH 64 // actually only CAN_QUEUE_LENGTH - 1 spaces can be used

typedef enum {
	CAN_SEND_BUSY,
	CAN_SEND_OK,
	CAN_SEND_EMPTY,
} CAN_send_status_e;

typedef struct {
	uint32_t std_id;
	uint8_t buf[8];
} CAN_message_t;

typedef struct {
	CAN_message_t queue[CAN_QUEUE_LENGTH];
	uint8_t head;
	uint8_t tail; // pointer to next empty space of the queue
	CAN_HandleTypeDef *hcan;
	int size;
} CAN_queue_t;

extern void can_send_init(void);
extern CAN_send_status_e can_send_message(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t buf[8]);

#endif
