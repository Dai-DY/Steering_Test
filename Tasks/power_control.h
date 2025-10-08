#ifndef POWER_CONTROL_H_
#define POWER_CONTROL_H_

#include "motor.h"
#include "comm.h"
#include "func_lib.h"
#include "pid.h"

#define POWER_CONTROL 1
#define POWER_TEST 0
#define POWER_SCALE_COMP 1
#define CURRENT_SCALE_COMP 1

void power_control_init(void);
void update_power_K(void);
void calc_cmd_power(void);
void power_control_main(void);

typedef struct{
  int16_t cur_speed[5];     //present speed of motor
  float cmd_power;          //predicted power
  float last_cmd_power;     //last predicted power
  float k;                  //parameters of low-pass filter
	float voltage_limit;      //the least voltage of Cap
	float buffer_to_power;    //in what level we change free buffer to power
	float max_power_limit;    //in what level we limit max_power when V_Cap is smaller than the limit
	float cur_chassis_power;  //the ture power of chassis by considering the charge of capacitor
	float cur_cap_voltage;    //the voltage of current capacitor
	float last_cap_voltage;		//the volatge of capacitor in last period
	float ema_power_to_walts; //EMA K from SPEED*CURRENT to Walts. 
	float cmd_power_output;   //cmd_power after times reduce_k, for test
	float max_power;
    float exceed_current;
	uint8_t rational_flag;
}power_control_t; 

typedef struct{   					// to calculated average power, for test
	float aver_pow;
	float total_pow;
	int aver;                 //counter
	float wheelspd1;
	float wheelspd2;
	float wheelspd3;
	float wheelspd4;
	float wheelcur1;
	float wheelcur2;
	float wheelcur3;
	float wheelcur4;
	float wheelcurk1;
	float wheelcurk2;
	float wheelcurk3;
	float wheelcurk4;
}test_t;

typedef struct{
    PID temp_pid[4];
}try_pid_t; 
#endif //POWER_CONTROL_H_
