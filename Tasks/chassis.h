#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "func_lib.h"
#include "robot_conf.h"

typedef enum {
	FOLLOW_ON_0_DEGREE = 1,
	FOLLOW_ON_45_DEGREE = 2,
	FOLLOW_OFF = 0,
} chassis_follow_t;

typedef struct {
	float car_yaw;
	float car_v;
} chassis_move_t;


typedef struct {
	float vx_set;
	float vy_set;
	float vw_set;
	chassis_follow_t follow;
	chassis_move_t chassis_move;
	float speed_set[5];
	float steering_angel_set[12];
} chassis_command_t;

typedef struct {
	float vx_set;
	float vy_set;
	float vw_set;
	chassis_follow_t follow;
	float speed_set[5];
	float speed_get[5];
	float vx_get;
	float vy_get;
	float vw_get;
	float gimbal_angle;
	float steering_decode_trans_angle[12];
	float steeringwheel_offset[12];
} chassis_info_t;

typedef struct {
	chassis_command_t command;
	chassis_info_t info;
	PID wheel_pid[5];
	FSFC steering_fsfc[12];
	PID rotate;
} chassis_t;

extern chassis_t chassis;



void chassis_task(void *args);
void chassis_off(void);
void chassis_info_get(chassis_info_t *info);
void chassis_info_update(void);
void chassis_set_command(float vx, float vy, float vw, chassis_follow_t follow);

#endif
