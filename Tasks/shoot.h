#ifndef __SHOOT_H__
#define __SHOOT_H__

#include "func_lib.h"
#include "robot_conf.h"
#include "os.h"

// for checking shoot block
#define BLOCK_TIME_THRESHOLD 70 // threshold of block time to determine that prwheel is blocked

// for count_fired_bullet to determine that a bullet is fired
#define FRIC_SPEED_DECREASE_INTEGRAL_THRESHOLD -6 // threshold of continuous decrease of fric speed
#define FRIC_SPEED_DECREASE_THRESHOLD 9 // threshold of decrease of fric speed relative to fric speed target
#define DIFFERENTIAL_THRESHOLD 0.1f // threshold of change of fric speed which is counted into fric_speed_decrease_integral

// for prwheel back set
#define BACK_SET_CURRENT 2000 // motor current for prwheel back set
#define BACK_SET_TIME  200 // the duation of prwheel back set

//threshold of overspeed
#define BULLET_OVERSPEED_THRESHOLD 15.8f

// command of shoot mode
typedef enum{
	SHOOT_COMMAND_OFF,
	SHOOT_COMMAND_SINGLE,
	SHOOT_COMMAND_CONTINUE,
} shoot_mode_command_e;

// states of shoot mode
typedef enum{
	SHOOT_STATE_OFF,
	SHOOT_STATE_SINGLE,
	SHOOT_STATE_CONTINUE,
	SHOOT_STATE_BLOCK, // prwheel back set when it is blocked
} shoot_mode_state_e;

// external, commander of shoot
typedef struct {
	shoot_mode_command_e shoot_mode_command;
	
	int fric_speed_set; // rpm before deceleration
	
	int prwheel_speed_set; // rpm after deceleration
} ext_shoot_command_t;

typedef struct {
	shoot_mode_state_e shoot_mode_command;
	
	int fric_speed_set; // rpm before deceleration
	
	int prwheel_speed_set; // rpm after deceleration
} shoot_execute_command_t;

// internal, information of shoot
typedef struct {
	// information about state
	shoot_mode_state_e shoot_mode_state;
	
	int	bullet_count; // counter of fired bullet
	
	float	cur_local_heat; // current local heat
	
	int block_flag; // whether it is checked that prwheeel is blocked
	
	int back_set_flag; // whether prwheel need to back set
	
	// helping variables
	int left_friction_speed_set; // rpm before deceleration
	int right_frction_speed_set; // rpm before deceleration
	
	float prwheel_speed_set; // rpm after deceleration
	
	int16_t left_friction_speed_get; // rpm before deceleration
	int16_t right_friction_speed_get; // rpm before deceleration
	
	int16_t prwheel_speed_get; // rpm after deceleration
	
	// for responsing bullet_count
	int	has_checked_bullet;
	
	// for calculating bullet_count
	float cur_fric_speed;
	float	last_fric_speed;
	float	differential;
	float	fric_speed_decrease_integral;
	float	cur_fric_spd_target;

	// for calculating cur_local_heat
	float cur_cal_local_heat_time;
	float last_cal_local_heat_time;

	// for calculating block_flag
	uint16_t block_time;
	
	uint32_t prwheel_back_set_start_time; // start time of prwheel back set
	
	int shoot_allow; // only for single shoot
	
	int bullet_overspeed_time;
} shoot_info_t;

typedef struct {
	PID left_friction_pid;
	PID right_friction_pid;
	
	PID prwheel_pid;
} shoot_motor_controller_t;

void shoot_task(void *args);
void handler_bullet(uint8_t* );
void shoot_set_command(shoot_mode_command_e shoot_mode_command_, int fric_speed_set_, int prwheel_speed_set_);
void shoot_info_get(shoot_info_t *dst);

#endif
