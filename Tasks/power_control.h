#ifndef _POWER_CONTROL_H
#define _POWER_CONTROL_H

#include "robot_conf.h"
#include "motor.h"
#include "comm.h"

#define REDUCTION_RATIO 19.20f
#define CURRENT_TO_AMPERE 20.f/16384.f
#define TORQUE_CONSTANT 0.3f/19.20f

#define RPM_TO_RADPS PI/30.f

#define MOTORS CAN1_Motors

#define THETA1_INIT 0.012f
#define THETA2_INIT 540.0f
#define K3_INIT 4.64f
// [TODO] rewrite after running on real environment
#define FORGETTING_FACTOR 0.999f

#define E_LOWER 20.0f
#define E_UPPER 80.0f

#define EPS 1e-6f

#define DBUF_TRIG -5.0f
#define BUF_THRESHOLD 40.0f

#define WITH_CAP 0


typedef struct {
	float speed;		// rad/s
	float current;	// A
	float torque;		// N*m
} meas_motor_info_t;

typedef struct {
	float speed;		// rad/s
	float err;			// rad/s
	float torque;		// N*m
	float P_cmd; 		// W
} cmd_motor_info_t;

typedef struct {
	float theta1;
	float theta2;
	float P11;
	float P12;
	float P22;
	float lambda; // forgetting factor
} RLS_t;

typedef struct {
	meas_motor_info_t motor[9];
	RLS_t rls_wheel;
	RLS_t rls_steering;
	
	float k1_w;		// factor of |w| loss for 3508
	float k2_w;		// factor of tau^2 loss for 3508
	float k1_s;		// factor of |w| loss for 6020
	float k2_s;		// factor of tau^2 loss for 6020
	float	k3;		// static power loss
	
	float sum_abs_w_wheel; 	// sum |w| for 3508
	float sum_tau2_wheel; 	// sum tau^2 for 6020
	float sum_tau_w_wheel; 	// sum tau*w = effective power for 3508
	
	float sum_abs_w_steering; 	// sum |w| for 3508
	float sum_tau2_steering; 	// sum tau^2 for 6020
	float sum_tau_w_steering; 	// sum tau*w = effective power for 3508
	
	uint8_t cap_on;
	uint8_t judge_on;
	float P_ref; 		// reference power limit
	float E_buf;		// reference chassis buffer
	
	float P_est;		// estimated power
	float P_meas;		// measured power
} power_info_t;

typedef struct {
	cmd_motor_info_t motor[9];
	
	float sum_abs_w_wheel; 	// sum |w| for 3508
	float sum_tau2_wheel; 	// sum tau^2 for 6020
	float sum_tau_w_wheel; 	// sum tau*w = effective power for 3508
	
	float sum_abs_w_steering; 	// sum |w| for 3508
	float sum_tau2_steering; 	// sum tau^2 for 6020
	float sum_tau_w_steering; 	// sum tau*w = effective power for 3508
	
	float P_cmd;
	
	float P_max[9];
	float alpha[9];			// perv alphas
	float filter_alpha;	// filter parameter
} power_cmd_t;

typedef struct {
	power_info_t info;
	power_cmd_t cmd;
	float current_set[9];
} power_control_t;

typedef struct {
	float E_buf;	// prev E_buf
	float err;		// prev err
	float Kp;
	float Kd;
	float E_set;
	float P_set;
	float filter_P;
} energy_control_t;

typedef struct {
	uint32_t count;
	float sum;
	float avg_k3;
} measure_k3_t;

#define MEASURE_ON 0
void power_measure_k3(void);

void power_control_init(void);
void power_control_info_update(void);
void power_control(void);
#endif

