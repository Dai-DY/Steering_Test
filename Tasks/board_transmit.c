/*	How to use?
		If you want to send message:
		1. add a struct typedef and header in board_transmit.h
		2. define a function in Packer section
		3. register the packer with header and priority in board_transmit_init()
		
		If you want to receive message:
		1. add a struct typedef and header in board_transmit.h
		2. define a variable in Data section
		3. define a function in Handler section
		4. register the handler with header in board_transmit_init()
		5. if you want a global variable, declare extern in board_transmit.h
	*/

#define _BOARD_TRANSMIT_C
#include "board_transmit.h"

extern thread_status_t thread_status;

/*================================Begin of Data ===========================================================*/
robot_state_trans_t robot_state_trans;
controller_channel_trans_t controller_channel_trans;
controller_others_trans_t controller_others_trans;
mosue_key_trans_t mouse_key_trans;
uint64_t trans_receive_time;

/*================================End of Data=============================================================*/

/*================================Begin of Packer Define =================================================*/

//void controller_channel_trans_packer(uint8_t buffer[8]){
//	memcpy(&buffer[0], &control.channel[0], sizeof(float));
//	memcpy(&buffer[4], &control.channel[1], sizeof(float));
//}


void judge_trans_packer(uint8_t buffer[8]){
	/*
	static float helper1;
	helper1 = shoot_data.bullet_speed;
	memcpy(&buffer[0], &helper1, sizeof(float));
	*/
	
	static uint16_t helper2;
	helper2 = reference_power_heat.shooter_id1_17mm_cooling_heat;
	memcpy(&buffer[4],&helper2,sizeof(uint16_t));
	
	static uint16_t helper3;
	helper3 =  reference_robot_state.shooter_barrel_heat_limit;
	memcpy (&buffer[6],&helper3,sizeof(uint16_t));
}

void robot_state_trans_packer(uint8_t buffer[8]){
	memcpy (&buffer[0], &reference_robot_state.robot_id, sizeof(uint8_t));
}

/*================================End of Packer Define =================================================*/

/*================================Begin of Handler Define =================================================*/


void controller_channel_trans_handler(uint8_t* data){
	memcpy(&controller_channel_trans, data, sizeof(controller_channel_trans_t));
	trans_receive_time = time();
}

void controller_others_trans_handler(uint8_t* data){
	memcpy(&controller_others_trans, data, sizeof(controller_others_trans_t));
	trans_receive_time = time();
}

void controller_mouse_key_trans_handler(uint8_t* data){
	memcpy(&mouse_key_trans, data, sizeof(mosue_key_trans_t));
	trans_receive_time = time();
}

/*================================End of Handler Define====================================================*/

void board_transmit_init(void) {
	/* register packer */
//	transmit_packer_register(CMD_CONTROLLER_CHANNEL_TRANS, controller_channel_trans_packer, transmitPriorityHigh);
	
	transmit_packer_register(CMD_JUDGE_TRANS,judge_trans_packer,transmitPriorityHigh);
	
	transmit_packer_register(CMD_ROBOT_STATE_TRANS, robot_state_trans_packer, transmitPriorityNormal);
	
	/* register handler */
	
	transmit_handler_register(CMD_CONTROLLER_CHANNEL_TRANS,controller_channel_trans_handler);
	
	transmit_handler_register(CMD_CONTROLLER_OTHERS_TRANS,controller_others_trans_handler);
	
	transmit_handler_register(CMD_MOUSE_KEY_TRANS,controller_mouse_key_trans_handler);

	osThreadCreate("board_transmit_task", board_transmit_task, NULL, osPriorityNormal, 128);
}

void board_transmit_task(void *args) {
    
	thread_status.cancom = THREAD_OK;
    while(!(thread_status.uartcom  && //waiting till all task threads are initialized
            thread_status.cancom   &&    
            thread_status.ui       &&
            thread_status.chassis  &&
            //thread_status.gimbal   &&
            //thread_status.shoot    &&
            thread_status.mainctrl )){
        delay(50);
    }
            
	while (1) {
		transmit_sender();
		delay(1);
	}
}

