#include "CAN_send.h"

extern CAN_HandleTypeDef hcan1, hcan2;

CAN_queue_t can1_queue;
CAN_queue_t can2_queue;

void can_send_init() {
	can1_queue.hcan = &hcan1;
	can1_queue.head = 0;
	can1_queue.tail = 0;
	
	can2_queue.hcan = &hcan2;
	can2_queue.head = 0;
	can2_queue.tail = 0;
	
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
}

int is_queue_full(CAN_queue_t *can_queue) {
	return ((can_queue->tail + 1) % CAN_QUEUE_LENGTH) == can_queue->head;
}

int is_queue_empty(CAN_queue_t *can_queue) {
	return can_queue->head == can_queue->tail;
}

CAN_send_status_e enqueue_can_message(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t buf[8]) {
	static CAN_queue_t *can_queue_object;

	if (hcan == &hcan1) {
		can_queue_object = &can1_queue;
	}
	else if (hcan == &hcan2) {
		can_queue_object = &can2_queue;
	}
	
	if (is_queue_full(can_queue_object)) {
		return CAN_SEND_BUSY;
	}
	
	can_queue_object->queue[can_queue_object->tail].std_id = std_id;
	memcpy(&can_queue_object->queue[can_queue_object->tail].buf, buf, (size_t)8);
	
	can_queue_object->tail = (can_queue_object->tail + 1) % CAN_QUEUE_LENGTH;
	
	can_queue_object->size++;
	
	return CAN_SEND_OK;
}

CAN_send_status_e can_send_message(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t buf[8]) {
	static CAN_TxHeaderTypeDef header = {.StdId = 0x0800, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8};
	static uint32_t *mailbox;
	
	if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
		header.StdId = std_id;
		if (HAL_CAN_AddTxMessage(hcan, &header, buf, mailbox) == HAL_OK) return CAN_SEND_OK;	
		
	}
	
	enqueue_can_message(hcan, std_id, buf);
	return CAN_SEND_OK;
}

CAN_send_status_e dequeue_can_message(CAN_HandleTypeDef *hcan) {
	static CAN_message_t message;
	static CAN_TxHeaderTypeDef header = {.StdId = 0x0800, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8};
	static uint32_t *mailbox;
	static CAN_queue_t *can_queue_object;
	
	if (hcan == &hcan1) {
		can_queue_object = &can1_queue;
	}
	else if (hcan == &hcan2) {
		can_queue_object = &can2_queue;
	}
	
	if (is_queue_empty(can_queue_object)) return CAN_SEND_EMPTY;
	
	message = can_queue_object->queue[can_queue_object->head];
	header.StdId = message.std_id;
		
	HAL_CAN_AddTxMessage(can_queue_object->hcan, &header, message.buf, mailbox);
	
	can_queue_object->head = (can_queue_object->head + 1) % CAN_QUEUE_LENGTH;
	
	can_queue_object->size--;
	
	return CAN_SEND_OK;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	dequeue_can_message(hcan);
}


void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	dequeue_can_message(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	dequeue_can_message(hcan);
}

