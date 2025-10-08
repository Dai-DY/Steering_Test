#ifndef _H_PERFORMANCE_TEST
#define _H_PERFORMANCE_TEST
#include "comm.h"
#include "robot_conf.h"
#include "math.h"

typedef struct {
	float pitch;
	float pitch_speed;
	float yaw;
	float yaw_speed;
}gimbal_test_t;


extern gimbal_test_t gimbal_test;
void Test_task(void *args);

#endif


