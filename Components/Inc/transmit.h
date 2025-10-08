#ifndef _TRANSMIT_H
#define _TRANSMIT_H
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "string.h"
#include "CAN_send.h"
#include "communicate.h"


#define CAN_TRANSMIT_ON 1
#define PACKER_REGISTRY_MAX_LENGTH 10
#define HANDLER_REGISTRY_MAX_LENGTH 60

#define PRIORITY_LOW_CYCLE 53
#define PRIORITY_NORMAL_CYCLE 31
#define PRIORITY_HIGH_CYCLE 17 // suitable for controller
#define TRANSMIT_CNT_CYCLE 500000


typedef enum{
	transmitPriorityLow,
	transmitPriorityNormal,
	transmitPriorityHigh
}transmitPriority_t;

typedef void (*packer_t)(uint8_t *buffer);
typedef void (*handler_t)(uint8_t *data);

typedef struct{
	uint16_t cmd_id;
	packer_t packer;
}packer_registry_t;

typedef struct{
	uint16_t cmd_id;
	handler_t handler;
}handler_registry_t;

void transmit_packer_register(uint16_t cmd_id, packer_t packer, transmitPriority_t priority);
void transmit_handler_register(uint16_t cmd_id, handler_t handler);

void transmit_sender(void);
void transmit_receiver(uint16_t cmd_id, uint8_t *data);

#endif
