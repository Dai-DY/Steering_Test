#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "func_lib.h"
#include "robot_conf.h"
#include "os.h"


// macros

#ifdef LIMIT_YAW
#define YAW_MOTOR_LOWER_BOUND 2000
#define YAW_MOTOR_UPPER_BOUND 4000
#endif

#define YAW_MOTOR_DIR -1
#define PITCH_MOTOR_DIR -1

#define PITCH_IMU_UPPER_BOUND 14*DEGREE_TO_MOTOR // maximum pitch angle, unit: motor
#define PITCH_IMU_LOWER_BOUND -14*DEGREE_TO_MOTOR // minimum pitch angle, unit: motor

#define YAW_FSFC_PARAMS		6000.0f, 11000.0f, 0.0f, 4000, 30000, 1

#define PITCH_FSFC_PARAMS		170.0f, 10000.0f, 0.0f, 4000, 30000, 1

#define PITCH_UPWARD_RANGE 26 // pitch angle range of upward (always positive), unit: deg
#define PITCH_DOWNWARD_RANGE -14 // pitch angle range of downward (always negative), unit: deg
#define PITCH_OFFSET 4296 // motor value when pitch angle equals zero

//	***** new module *****

typedef struct {
	float yaw_angle_set;
	float yaw_speed_set;
	
	float pitch_angle_set;
	float pitch_speed_set;
} gimbal_command_t;

typedef struct {
	float yaw_angle_set;
	float yaw_speed_set;
	float pitch_angle_set;
	float pitch_speed_set;
	
	float yaw_angle_get;
	float yaw_speed_get;
	float pitch_angle_get;
	float pitch_speed_get;
	
	uint16_t yaw_motor_angle_get;
	float yaw_upper_bound;
	float yaw_lower_bound;
	
	float pitch_upper_bound;
	float pitch_lower_bound;
} gimbal_info_t;

typedef struct {
	gimbal_command_t command;
	gimbal_info_t info;
	
	FSFC yaw_fsfc;
	FSFC pitch_fsfc;
} gimbal_t;

void gimbal_task(void *args);

void gimbal_info_get(gimbal_info_t *info);
void gimbal_set_command(float yaw_angle, float yaw_speed, float pitch_angle, float pitch_speed);

#endif
