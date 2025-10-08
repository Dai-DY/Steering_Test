#include "transmit.h"

extern CAN_HandleTypeDef hcan1, hcan2;
extern UART_HandleTypeDef huart1, huart3, huart6;

packer_registry_t priorityLow_packer_registry[PACKER_REGISTRY_MAX_LENGTH];
packer_registry_t priorityNormal_packer_registry[PACKER_REGISTRY_MAX_LENGTH];
packer_registry_t priorityHigh_packer_registry[PACKER_REGISTRY_MAX_LENGTH];
uint16_t priorityLow_packer_registry_length = 0;
uint16_t priorityNormal_packer_registry_length = 0;
uint16_t priorityHigh_packer_registry_length = 0;

handler_registry_t handler_registry[HANDLER_REGISTRY_MAX_LENGTH];
uint16_t handler_registry_length = 0;

uint32_t transmit_cnt = 0;

void transmit_packer_register(uint16_t cmd_id, packer_t packer, transmitPriority_t priority){
	switch(priority){
		case transmitPriorityLow:
			if(priorityLow_packer_registry_length >= PACKER_REGISTRY_MAX_LENGTH) return ;
			priorityLow_packer_registry[priorityLow_packer_registry_length] = (packer_registry_t){
				.cmd_id = cmd_id, .packer = packer};
			priorityLow_packer_registry_length ++;
			break;
				
		case transmitPriorityNormal:
			if(priorityNormal_packer_registry_length >= PACKER_REGISTRY_MAX_LENGTH) return ;
			priorityNormal_packer_registry[priorityNormal_packer_registry_length] = (packer_registry_t){
				.cmd_id = cmd_id, .packer = packer};
			priorityNormal_packer_registry_length ++;
			break;
				
		case transmitPriorityHigh:
			if(priorityHigh_packer_registry_length >= PACKER_REGISTRY_MAX_LENGTH) return ;
			priorityHigh_packer_registry[priorityHigh_packer_registry_length] = (packer_registry_t){
				.cmd_id = cmd_id, .packer = packer};
			priorityHigh_packer_registry_length ++;
			break;
				
		default:
			break;
	}
}

void transmit_handler_register(uint16_t cmd_id, handler_t handler){
	if(handler_registry_length >= HANDLER_REGISTRY_MAX_LENGTH) return ;
	handler_registry[handler_registry_length] = (handler_registry_t){
		.cmd_id = cmd_id, .handler = handler};
	handler_registry_length ++;
}

void CAN_transmit_sender(CAN_HandleTypeDef *hcan, uint32_t *mailbox, uint8_t buf[8], const packer_registry_t *packer_registry, const uint16_t length){
	for(int i = 0; i < length; i ++){
		uint32_t StdId = packer_registry[i].cmd_id;
		
		packer_registry[i].packer(buf);
		can_send_message(hcan, StdId, buf);
	}
}

void uart_transmit_sender(UART_HandleTypeDef *uart, const packer_registry_t *packer_registry, const uint16_t length){
	uint8_t buf[8];
	for(int i = 0; i < length; i ++){
		packer_registry[i].packer(buf);
		comm_transmit_data(uart, 0xa5, packer_registry[i].cmd_id, buf, sizeof(buf));
	}
}

void transmit_sender(void){
	#if CAN_TRANSMIT_ON
	static uint8_t buf[8];
	static CAN_HandleTypeDef *hcan = &hcan2;
	static uint32_t mailbox;
	
	if(transmit_cnt % PRIORITY_HIGH_CYCLE == 0) CAN_transmit_sender(hcan, &mailbox, buf, priorityHigh_packer_registry, priorityHigh_packer_registry_length);
	if(transmit_cnt % PRIORITY_NORMAL_CYCLE == 0) CAN_transmit_sender(hcan, &mailbox, buf, priorityNormal_packer_registry, priorityNormal_packer_registry_length);
	if(transmit_cnt % PRIORITY_LOW_CYCLE == 0) CAN_transmit_sender(hcan, &mailbox, buf, priorityLow_packer_registry, priorityLow_packer_registry_length);
	#else
	static UART_HandleTypeDef *huart = &huart1;
	if(transmit_cnt % PRIORITY_HIGH_CYCLE == 0) uart_transmit_sender(huart, priorityHigh_packer_registry, priorityHigh_packer_registry_length);
	if(transmit_cnt % PRIORITY_NORMAL_CYCLE == 0) uart_transmit_sender(huart, priorityNormal_packer_registry, priorityNormal_packer_registry_length);
	if(transmit_cnt % PRIORITY_LOW_CYCLE == 0) uart_transmit_sender(huart, priorityLow_packer_registry, priorityLow_packer_registry_length);
	#endif
	
	transmit_cnt ++;
	if(transmit_cnt >= TRANSMIT_CNT_CYCLE) transmit_cnt = 0;
}

void transmit_receiver(uint16_t cmd_id, uint8_t *data){
	for(int i = 0; i < handler_registry_length; i ++){
		if(handler_registry[i].cmd_id == cmd_id){
			handler_registry[i].handler(data);
			return ;
		}
	}
}
