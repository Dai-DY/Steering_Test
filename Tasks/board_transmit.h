#ifndef __BOARD_TRANSMIT_H__
#define __BOARD_TRANSMIT_H__

#include "func_lib.h"
#include "robot_conf.h"
#include "os.h"
#include "transmit.h"
#include "main_control.h"

/*=========================================Begin of Users' Data TypeDef ========================================= */
#define CMD_CONTROL_MODE_TRANS 0x0401
typedef struct __attribute__((packed)) {
	uint8_t control_mode;
	int fric_on;
}control_mode_trans_t;

#define CMD_CONTROLLER_CHANNEL_TRANS 0x0402
typedef struct __attribute__((packed)) {
	float channel_0;
	float channel_1;
}controller_channel_trans_t;

#define CMD_CONTROLLER_OTHERS_TRANS 0x0403
typedef struct __attribute__((packed)) {
	uint8_t triswitch_0;
	uint8_t triswitch_1;
	uint8_t online;
}controller_others_trans_t;

#define CMD_SLAM_CONTROL_TRANS 0x0404
typedef struct __attribute__((packed)) {
	float vx;
	float vy;
}slam_control_trans_t;

#define CMD_ROBOT_STATE_TRANS 0x0405
typedef struct __attribute__((packed)) {
	uint8_t robot_id; 
}robot_state_trans_t;

#define CMD_MOUSE_KEY_TRANS 0x0407
typedef struct __attribute__((packed)) {
	uint16_t controller_key;
}mosue_key_trans_t;

#define CMD_JUDGE_TRANS 0x0408
typedef struct __attribute__((packed)){
	float bullet_speed;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t  shooter_barrel_heat_limit;
}judge_trans_t;
/*=========================================End of Users' Data TypeDef   ========================================= */

extern robot_state_trans_t robot_state_trans;
extern controller_channel_trans_t controller_channel_trans;
extern controller_others_trans_t controller_others_trans;
extern mosue_key_trans_t mouse_key_trans;

void board_transmit_task(void *);
void board_transmit_init(void);
#endif
