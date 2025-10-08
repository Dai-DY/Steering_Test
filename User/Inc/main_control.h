#ifndef _MAIN_CONTROL_H_
#define _MAIN_CONTROL_H_

#include "func_lib.h"
#include "robot_conf.h"
#include "chassis.h"
#include "shoot.h"
#include "gimbal.h"
#include "power_control.h"

// for controller
/// L: control.triSwitch[0]  R: control.triSwitch[1]
#define L    0
#define R    1

/// for 3 states of control.triSwitch
#define UP   1
#define MID  3
#define DOWN 2 

// distinct speed target of prwheel
#define PRWHEEL_EXPECTED_SPD_FAST 200 // for fast continue shoot
#define PRWHEEL_EXPECTED_SPD_SLOW 75 // for slow continue shoot
#define PRWHEEL_EXPECTED_SPD_SINGLE 60 // for single shoot
#define PRWHEEL_EXPECTED_SPD_FOR_CV_DEBUG 40 // for single shoot when CV_DEBUG

#define FRIC_SPD_25 6600 // speed of friction wheel before deceleration

#define BULLET_SPD 24.0f // speed of bullet, m/s

#define YAW_INC -250.0f / 360.0f // increase speed of yaw when controlled by contoller
#define PIT_INC -1.0f / 360.0f * 8192.0f // increase speed of pitch when controlled by contoller

#define YAW_SCANNING_SPEED 0.45f // speed of yaw scanning
#define YAW_SEARCHING_SPEED 1.5f

#define lvl_max 1
#define keyboard_spd_ratio 0.425f
#define spd_ratio 6.0f
#define keymouse_spd_ratio 2.0f
#define FAST_ROTATE 1.5f


#define _AC_spd 2.0f
#define _SL_spd 0.5f


typedef enum{
    THREAD_INIT,
    THREAD_OK,
    THREAD_FAULT,
} thread_status_e;

typedef struct{
    
    thread_status_e uartcom;
    thread_status_e cancom;
    thread_status_e ui;
    thread_status_e chassis;
    thread_status_e gimbal;
    thread_status_e shoot;
    thread_status_e mainctrl;
    
} thread_status_t;

//states of keymouse
typedef struct{
	uint8_t top_enable;
	uint8_t chassis_following;
	uint8_t chassis_sp[4];
	uint8_t c_key_cd;
	uint8_t z_key_cd;
	uint8_t Top_enable;
	uint8_t Top_enable_fast;
	uint8_t r_key_pressed;
	uint8_t shift_mode;
	uint8_t z_key_pressed;
	uint8_t shift_key_pressed;
	uint8_t x_key_pressed;
} control_mode_t;


typedef struct{
	float         W_lvl;
	float         S_lvl;
	float         A_lvl;
	float         D_lvl;
}kbd_spd_t;


// states of control mode of the whole robot
typedef enum{
	OFF_MODE,
	CONTROLLER_MODE,
	MOUSE_KEY_MODE,
} control_mode_e;

// states of chassis control mode
typedef enum {
	CONTROLLER_ALONE, // controlled by controller
	TOP_WITH_CONTROLLER, // controlled by controller while topping
	TOP_ALONE, // only top
	CHASSIS_OFF, // off
	MOUSE_KEY_CONTROL,
	MOUSE_KEY_FOLLOW,
	MOUSE_KEY_FOLLOW1,
	MOUSE_KEY_FOLLOW2,
	MOUSE_KEY_ROTATE,
	MOUSE_KEY_ROTATE_FAST,
} chassis_control_mode_e;

typedef enum {
	OFF,
	SINGLE,
	CONTINUE,
} shoot_mode_e;

typedef enum{
    TOP_OFF = 0,
    TOP_CW = 1,
    TOP_CCW = -1,

} top_mode_e;
// states of gimbal mode
typedef enum {
	KEEP_STILL,
	AUTO_AIM,
	SCANNING,
	CONTROLLER_CONTROLLED,
	DEFENSE,
	UNIFORM_NOD, // for pitch gravity compensation adjust
	CONTROLLED_BY_NAV,
} gimbal_mode_e;

typedef enum {
	FOWARD_MODE,
	BACK_MODE,
} direction_mode_e;

// internal, information of mode including all modes
typedef struct {
	// informtion about mode
	control_mode_e control_mode;
	
	chassis_control_mode_e chassis_control_mode;
    
    top_mode_e top_mode;
	
	int fric_on; // whether friction wheel is on
	
	shoot_mode_e shoot_mode;
	
	gimbal_mode_e gimbal_mode;
	
	direction_mode_e direction_mode;
	
	// helping variables which determine shoot mode
	int heat_spare; // whether heat is spare for shoot
	int start_time;
	int chassis_stop;
    
} main_control_mode_t;

// internal, controller of motor including pid, fsfc and feedforward
typedef struct {
	PID wheel_pids[5];
	
	
	
	FSFC yaw_fsfc;
	
	FSFC pitch_fsfc;
	float pitch_gravity_compensation_current; // current of pitch gravity compensation (feedforward)
} motor_controller_t;


void main_control_task(void*);
extern main_control_mode_t main_control_mode;
extern motor_controller_t motor_controller;
#endif

