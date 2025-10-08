/*	How to use?
		1. you can only use function in Interface section
		2. you can see enum or struct of argument in shoot.h
		3. you can modify macro define about shoot in robot_conf.h
	*/

#define _SHOOT_C

#include "shoot.h"
#include "comm.h"

extern thread_status_t thread_status;

// external, read only, only entry to control shoot
ext_shoot_command_t ext_shoot_command;

shoot_execute_command_t shoot_execute_command;

// internal, information of shoot
shoot_info_t int_shoot_info;

shoot_motor_controller_t shoot_motor_controller;

// external, write only, listener of shoot data for communication
ext_shoot_data_t shoot_data;

// **************************************** Begin of Interface ****************************************//
void shoot_set_command(shoot_mode_command_e shoot_mode_command_, int fric_speed_set_, int prwheel_speed_set_) {
	ext_shoot_command.shoot_mode_command = shoot_mode_command_;
	ext_shoot_command.fric_speed_set = fric_speed_set_;
	ext_shoot_command.prwheel_speed_set = prwheel_speed_set_;
}

void shoot_info_get(shoot_info_t *dst) {
	memcpy(dst, &int_shoot_info, sizeof(shoot_info_t));
}

// **************************************** End of Interface ****************************************//

// ****************************************** Begin of Init ******************************************//
void shoot_init() {
	ext_shoot_command.fric_speed_set = 0;
	ext_shoot_command.shoot_mode_command = SHOOT_COMMAND_OFF;
	ext_shoot_command.prwheel_speed_set = 0;
	
	shoot_execute_command.fric_speed_set = 0;
	shoot_execute_command.shoot_mode_command = SHOOT_STATE_OFF;
	shoot_execute_command.prwheel_speed_set = 0;
	
	int_shoot_info.shoot_mode_state = SHOOT_STATE_OFF;
	int_shoot_info.left_friction_speed_set = 0;
	int_shoot_info.right_frction_speed_set = 0;
	int_shoot_info.bullet_count = 0;
	int_shoot_info.has_checked_bullet = 0;
	int_shoot_info.cur_fric_speed = 0;
	int_shoot_info.last_fric_speed = 0;
	int_shoot_info.differential = 0;
	int_shoot_info.fric_speed_decrease_integral = 0;
	int_shoot_info.cur_fric_spd_target = 0;
	int_shoot_info.cur_local_heat = 0;
	int_shoot_info.cur_cal_local_heat_time = 0;
	int_shoot_info.last_cal_local_heat_time = 0;
	int_shoot_info.prwheel_speed_set = 0;
	int_shoot_info.block_flag = 0;
	int_shoot_info.block_time = 0;
	int_shoot_info.bullet_overspeed_time = 0;
	int_shoot_info.back_set_flag = 0;
	int_shoot_info.left_friction_speed_get = 0;
	int_shoot_info.right_friction_speed_get = 0;
	int_shoot_info.prwheel_speed_get = 0;
	int_shoot_info.prwheel_back_set_start_time = 0;
	int_shoot_info.shoot_allow = 0;
	
	PID_init(&(shoot_motor_controller.left_friction_pid), 20,0,0,500,2000,14000,0,1);
	PID_init(&(shoot_motor_controller.right_friction_pid), 20,0,0,500,2000,14000,0,1);
	PID_init(&(shoot_motor_controller.prwheel_pid), 120,0.03,27,1000,2000,10000,0,1);
    
	thread_status.shoot = THREAD_OK;
    while(!(thread_status.uartcom  && //waiting till all task threads are initialized
            thread_status.cancom   &&    
            thread_status.ui       &&
            thread_status.chassis  &&
            //thread_status.gimbal   &&
            //thread_status.shoot    &&
            thread_status.mainctrl )){
        delay(50);
    }
	CanMotor_ResetTotalAngle(&CAN2_Motors[PRWHEEL_ID]);
}

// ****************************************** End of Init ******************************************//

// **************************************** Begin of SubTask about update_shoot_state *****************************************//
/**
* @breif   count the number of fired bullet
* @param   none
* @return  none
* @note		 shoot.bullet_count is usually at most 1, due to the function cal_cur_local_heat().
*/
void count_fired_bullet() {
	int_shoot_info.cur_fric_spd_target = shoot_execute_command.fric_speed_set / M3508_DECELE_RATIO;
	int_shoot_info.cur_fric_speed = abs(CAN2_Motors[FRIC2_ID].speed)/M3508_DECELE_RATIO;
	
	if (shoot_execute_command.fric_speed_set && int_shoot_info.cur_fric_speed > int_shoot_info.cur_fric_spd_target * 0.8f) {
		int_shoot_info.differential = int_shoot_info.cur_fric_speed - int_shoot_info.last_fric_speed;
		
		// If differential of friction speed approximately < 0 (which means we can give DIFFERENTIAL_THRESHOLD a slightly positive value to avoid positive change of friction speed),
		// it is considered to be in a probably continuous decreasing.
		if (int_shoot_info.differential < DIFFERENTIAL_THRESHOLD) {
			int_shoot_info.fric_speed_decrease_integral += int_shoot_info.differential;
		}
		else {
			// Only when friction wheel speed constantly decreases and last_fric_speed drops down to some value, a bullet is fired.
			if (int_shoot_info.fric_speed_decrease_integral < FRIC_SPEED_DECREASE_INTEGRAL_THRESHOLD && int_shoot_info.last_fric_speed < int_shoot_info.cur_fric_spd_target - FRIC_SPEED_DECREASE_THRESHOLD) {
				int_shoot_info.bullet_count++;
			}
			int_shoot_info.fric_speed_decrease_integral = 0;
		}
	}
	else {
		int_shoot_info.fric_speed_decrease_integral = 0;
	}
	
	int_shoot_info.last_fric_speed = int_shoot_info.cur_fric_speed;
}

/**
* @breif   calculate current local heat
* @param   none
* @return  none
* @note		 Instead of using the heat data from judgement, we can know smoother and more real-time heat data 
*					 to avoid exceeding heat limit when continuous shooting in a high frequency.
*/
void cal_cur_local_heat() {
	int_shoot_info.cur_cal_local_heat_time = time() / 1000.0; //s

	int_shoot_info.cur_local_heat += int_shoot_info.bullet_count * HEAT_PER_SHOOT 
													- (int_shoot_info.cur_cal_local_heat_time - int_shoot_info.last_cal_local_heat_time) * reference_power_heat.shooter_id1_17mm_cooling_heat;

	if (int_shoot_info.cur_local_heat < 0) int_shoot_info.cur_local_heat = 0;
	
	int_shoot_info.bullet_count = 0;
	
	int_shoot_info.last_cal_local_heat_time = int_shoot_info.cur_cal_local_heat_time;
}

/**
* @breif   check block with a block counter
* @param   none
* @return  none
* @note		 
*/
void check_block() {
	// Only when prwheel is pulling bullets, we need to check block.
	if (int_shoot_info.back_set_flag || shoot_execute_command.prwheel_speed_set == 0) return;
	
	float spd_get = fabs(CAN2_Motors[PRWHEEL_ID].speed / M2006_REDUCTION_RATIO);
	
	if(spd_get < shoot_execute_command.prwheel_speed_set * 0.05) int_shoot_info.block_time ++;
	else int_shoot_info.block_time = 0;

	// If speed of prwheel cannot reach the target for a continuous time, we consider the prwheel is blocked.
	if(int_shoot_info.block_time > BLOCK_TIME_THRESHOLD){
		int_shoot_info.block_flag = 1;
		int_shoot_info.block_time = 0;
	}
	else int_shoot_info.block_flag = 0;
}

// **************************************** End of SubTask about update_shoot_state *****************************************//

// **************************************** Begin of SubTask of process_command *****************************************//
/**
* @breif    update state of shoot
* @param		none
* @return		none
* @note			
*/
void update_shoot_state() {
	count_fired_bullet();
	
	// record whether bullet has been checked
	if (int_shoot_info.bullet_count) {
		int_shoot_info.has_checked_bullet = 1;
	}
	else {
		int_shoot_info.has_checked_bullet = 0;
	}
	
	cal_cur_local_heat();

	check_block();
}

void update_shoot_allow() {
	// If a bullet is fired when single shoot, it is not allowed to fire another bullet.
	if (int_shoot_info.has_checked_bullet && int_shoot_info.shoot_mode_state == SHOOT_STATE_SINGLE) {
			int_shoot_info.shoot_allow = 0;
	}
}

/**
* @breif   update back_set_flag
* @param   none
* @return  none
* @note		 
*/
void back_set_judgement() {
	if (int_shoot_info.back_set_flag) {
		// After prwheel has back set for some time, it needn't back set.
		if (time() - int_shoot_info.prwheel_back_set_start_time > BACK_SET_TIME) {
			int_shoot_info.back_set_flag = 0;
		}
	}
	else {
		if (!int_shoot_info.block_flag) {
			int_shoot_info.back_set_flag = 0;
		}
		// If prwheel is checked to be blocked, it need to back set.
		else {
			int_shoot_info.back_set_flag = 1;
			int_shoot_info.prwheel_back_set_start_time = time();
		}
	}
}

// **************************************** End of SubTask of process_command *****************************************//

// **************************************** Begin of SubTask of main task *****************************************//
void process_command() {
	update_shoot_state();
	
	update_shoot_allow();
	
	back_set_judgement();
	
	if (ext_shoot_command.fric_speed_set < 0 || ext_shoot_command.prwheel_speed_set < 0) {
		shoot_execute_command.shoot_mode_command = SHOOT_STATE_OFF;
		shoot_execute_command.fric_speed_set = 0;
		shoot_execute_command.prwheel_speed_set = 0;
		return;
	}
	
	if (ext_shoot_command.fric_speed_set == 0 && ext_shoot_command.prwheel_speed_set > 0) {
		shoot_execute_command.shoot_mode_command = SHOOT_STATE_OFF;
		shoot_execute_command.fric_speed_set = 0;
		shoot_execute_command.prwheel_speed_set = 0;
		return;
	}
	
	if (int_shoot_info.back_set_flag) {
		shoot_execute_command.shoot_mode_command = SHOOT_STATE_BLOCK;
		shoot_execute_command.fric_speed_set = ext_shoot_command.fric_speed_set;
		shoot_execute_command.prwheel_speed_set = 0;
		return;
	}
	
	switch (ext_shoot_command.shoot_mode_command) {
		case SHOOT_COMMAND_CONTINUE:
			shoot_execute_command.shoot_mode_command = SHOOT_STATE_CONTINUE;
			shoot_execute_command.fric_speed_set = ext_shoot_command.fric_speed_set;
			shoot_execute_command.prwheel_speed_set = ext_shoot_command.prwheel_speed_set;
			break;
			
		case SHOOT_COMMAND_SINGLE:
			if (int_shoot_info.shoot_allow) {
				shoot_execute_command.shoot_mode_command = SHOOT_STATE_SINGLE;
				shoot_execute_command.fric_speed_set = ext_shoot_command.fric_speed_set;
				shoot_execute_command.prwheel_speed_set = ext_shoot_command.prwheel_speed_set;
			}
			else {
				shoot_execute_command.shoot_mode_command = SHOOT_STATE_OFF;
				shoot_execute_command.fric_speed_set = ext_shoot_command.fric_speed_set;
				shoot_execute_command.prwheel_speed_set = 0;
			}
			break;
			
		case SHOOT_COMMAND_OFF:
			shoot_execute_command.shoot_mode_command = SHOOT_STATE_OFF;
			shoot_execute_command.fric_speed_set = ext_shoot_command.fric_speed_set;
			shoot_execute_command.prwheel_speed_set = 0;
		
			int_shoot_info.shoot_allow = 1;
			break;
		
		default:
			shoot_execute_command.shoot_mode_command = SHOOT_STATE_OFF;
			shoot_execute_command.fric_speed_set = 0;
			shoot_execute_command.prwheel_speed_set = 0;
			break;
	}
}

void update_shoot_mode_state() {
	int_shoot_info.shoot_mode_state = shoot_execute_command.shoot_mode_command;
}

void calculate_fric_speed() {
	int_shoot_info.left_friction_speed_set = shoot_execute_command.fric_speed_set * FRIC_DIR; // RPM before deceleration
	int_shoot_info.right_frction_speed_set = -1 * shoot_execute_command.fric_speed_set * FRIC_DIR; // RPM before deceleration
}

/**
* @breif   calculate speed of prwheel
* @param   none
* @return  none
* @note	   If prwheel need to back set, we directly use a certain current to control it (see in main_control).
*/
void calculate_prwheel_speed(){
	int_shoot_info.prwheel_speed_set = shoot_execute_command.prwheel_speed_set * PRWHEEL_DIR; // RPM after deceleration
}

void update_motor_state() {
	int_shoot_info.left_friction_speed_get = CAN2_Motors[FRIC1_ID].speed; // RPM before deceleration
	int_shoot_info.right_friction_speed_get = CAN2_Motors[FRIC2_ID].speed; // RPM before deceleration
	
	int_shoot_info.prwheel_speed_get = CAN2_Motors[PRWHEEL_ID].speed/M2006_REDUCTION_RATIO; // RPM after deceleration
}

void execute_shoot() {
	// calculate current
	if (control.online == 0) {
		CAN2_MotorCurrents[FRIC1_ID] = 0;
		CAN2_MotorCurrents[FRIC2_ID] = 0;
		CAN2_MotorCurrents[PRWHEEL_ID] = 0;
	}
	else {
		CAN2_MotorCurrents[FRIC1_ID] = PID_calc(&shoot_motor_controller.left_friction_pid, int_shoot_info.left_friction_speed_get, int_shoot_info.left_friction_speed_set);
		CAN2_MotorCurrents[FRIC2_ID] = PID_calc(&shoot_motor_controller.right_friction_pid, int_shoot_info.right_friction_speed_get, int_shoot_info.right_frction_speed_set);
		
		// If prwheel need to back set, we directly use a certain current to control it.
		if (int_shoot_info.shoot_mode_state == SHOOT_STATE_BLOCK) {
			CAN2_MotorCurrents[PRWHEEL_ID] = -1 * PRWHEEL_DIR * BACK_SET_CURRENT;
		}
		else {
			CAN2_MotorCurrents[PRWHEEL_ID] = PID_calc(&shoot_motor_controller.prwheel_pid, int_shoot_info.prwheel_speed_get, int_shoot_info.prwheel_speed_set);
		}
	}
	
	CanMotor_SendCurrent(CAN2_MOTOR_1_TO_4);
	CanMotor_SendCurrent(CAN2_MOTOR_5_TO_8);
}

// **************************************** End of SubTask of main task *****************************************//

// **************************************** Begin of MainTask ****************************************//
void shoot_task(void *args){
	shoot_init();
	while (1) {
		process_command();
		
		update_shoot_mode_state();
		calculate_fric_speed();
		calculate_prwheel_speed();
		
		update_motor_state();
		execute_shoot();
		
		delay(5);
	}
}

// **************************************** End of MainTask ****************************************//

// **************************************** Begin of Listener ****************************************//
/**
* @breif    update shoot_data
* @param		data
* @return		none
* @note			Used for communication.
*/
void handler_bullet(uint8_t* data){
	memcpy(&shoot_data,data,sizeof(shoot_data));
	
	if (shoot_data.bullet_speed > BULLET_OVERSPEED_THRESHOLD) {
		int_shoot_info.bullet_overspeed_time++;
	}
}

// **************************************** End of Listener ****************************************//
