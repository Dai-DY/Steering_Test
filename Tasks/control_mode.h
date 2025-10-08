#ifndef CONTROL_MODE_H
#define CONTROL_MODE_H

#include "func_lib.h"
#include "robot_conf.h"

/**********CONTENT**********/
// 0. Header
// 1. Controller Status Definition
// 2. Enumly Define Control Modes of Robot 
// 		(Include: control_mode_e - based on status of controller
//							gim_chassis_e  - present the relationship between gimbal and chassis -- follow or not
//							shoot_mode_e   - mode of shooting, set above shoot.c; OFF, SINGLE and CONTINUE - TRIPLE for special use)
// 3. Mode Flag - flagged when control modes change
// 4. Control Mode Definition - Combination of all modes and function flags




/// L: control.triSwitch[0]  R: control.triSwitch[1]
#define L    0
#define R    1

/// control.triSwitch 3 kinds of status
#define UP   1
#define MID  3
#define DOWN 2 
/// RAGE_MODE 
#define RAGE_MODE 0



//*******************************************************//
/**
* @brief  this enum defines 4 control mode
* @note		OFF_MODE: 					controller offline
*         MOUSE_KEY_MODE: 		control robots with mouse and key
*         CONTROLLER_MODE:    ~MODE1 for MID; ~MODE2 for UP
*/
typedef enum{
	OFF_MODE,
	MOUSE_KEY_MODE,
	CONTROLLER_MODE1,
	CONTROLLER_MODE2,
} control_mode_e;

/**
* @brief this enum define 4 modes which the gimbal and chassis can be
* @note  GIMBAL_FOLLOW_CHASSIS:    
*				 GIMBAL_CHASSIS_SEPERATE:   
*				 CHASSIS_FOLLOW_GIMBAL:     yaw 
*				 TOP_MODE:									top on
*/
typedef enum{
	GIMBAL_FOLLOW_CHASSIS,
	GIMBAL_CHASSIS_SEPERATE,
	CHASSIS_FOLLOW_GIMBAL,
	TOP_MODE,
	AUTO_CRUISE_MODE
} gim_chassis_e;

/**
* @brief this enum defines 3 modes of shoot
* @note	 OFF:        no shoot
*        SINGLE:     shoot once
*				 CONTINUE:   keep shooting 
*/
typedef enum{
	OFF,
	SINGLE,
	//TRIPLE,
	CONTINUE
}shoot_mode_e;
 



/**
* @brief this struct contains variables to react when control modes are going to change
* @note	 fric_flag:         define fric should be ON or OFF
*				 shoot_flag:				enable the switch of fric and shoot
*				 magazine_flag:			control magazine status
*			   heat_flag:					define overheating status
*				 heat_judge_flag: 	powercontrol swtich
*/
typedef struct{
	uint8_t fric_flag;
	uint8_t shoot_flag;
	uint8_t magazine_flag;
	uint8_t heat_flag;
	uint8_t heat_judge_flag;
}flag_t;

/**
* @brief this struct contains the parameters of all mode of the robot
* @note	 
*/
typedef struct{
	control_mode_e mode;
	gim_chassis_e gim_chassis;
	shoot_mode_e  shoot_mode;
	uint8_t fric_spd_lvl;
	uint8_t fric_enable;
	uint8_t magazine_on;
	uint8_t AUTO_AIM;
	uint8_t singleShoot;
	uint8_t heat_judge_mode;
	uint8_t chassis_following;
	uint8_t chassis_sp;
	
	uint8_t q_key_pressed;
	uint8_t f_key_pressed;
	uint8_t m_key_pressed;
	uint8_t g_key_pressed;
	uint8_t rightButton_pressed;
	uint8_t r_key_pressed;
	uint8_t auto_cruise;
	uint8_t shift_mode;
	uint8_t z_key_pressed;
	uint8_t shift_key_pressed;
	uint8_t gim_shift_status;
	uint8_t tunnel_mode;
	uint8_t turn_mode;
} control_mode_t;



extern control_mode_t control_mode;

void control_mode_task(void *args);

#endif
