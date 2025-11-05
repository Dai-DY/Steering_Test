#define _CHASSIS_C_

#include "chassis.h"
#include "power_control.h"
#include "math.h"

extern thread_status_t thread_status;
extern power_control_t power;

chassis_t chassis;

int init_delta_angle = 0;
float chassic_angel = 0.0f;
float gimbal_angel = 0.0f;
float YAW_ANGLE = 0.0f;
float t_curAngle;
FSFC YAW_Dir;
// ***************************************** Begin of Helping *****************************************//
/**
* @breif  calculate the angle between the direction of one armor and gimbal
* @param  t_init_delta_angle: delta angle between the direction of the armor and 0 degree of totalangle
* @return -t_curAngle:		0~180 deg or 0~-180 deg relative to the direction of the armor
* @note   When rotating clockwise 0~180 deg, it returns a positive degree.(just opposite to the sign of totalAngle)
*					When rotating anticlockwise 0~180 deg, it returns a negative degree.
*/
float chassisAngle(){

	if(CAN1_Motors[YAW_ID].totalAngle+init_delta_angle>0)    
		t_curAngle = (float)((int)(CAN1_Motors[YAW_ID].totalAngle+init_delta_angle) % 8192) / 22.7556f;
	else{
		t_curAngle = -(float)((int)(-CAN1_Motors[YAW_ID].totalAngle-init_delta_angle) % 8192) / 22.7556f;
	}
//    if(t_curAngle > 360 && t_curAngle <= 720)
//        t_curAngle = t_curAngle - 360;
//    if(t_curAngle < 0 && t_curAngle >= -360)
//        t_curAngle = t_curAngle + 360;
	if(t_curAngle > 180 && t_curAngle <= 540)
		t_curAngle = t_curAngle - 360;
  if(t_curAngle < -180 && t_curAngle >= -540)
		t_curAngle = t_curAngle + 360;
	
	return t_curAngle;
}

float gimbal_Angle(float off_set){
	return (CAN1_Motors[YAW_ID].totalAngle - off_set) /22.7556f;
}


float calculateShortestDistance(float yaw_now_360, float yaw_set_360,float* reverflag) {
    float clockwise_distance = fmodf((yaw_set_360 - yaw_now_360 + 360), 360);
    float counter_clockwise_distance = 360 - clockwise_distance;
    float reverse_distance = fabsf(fmodf(yaw_set_360 - yaw_now_360 + 180, 360)) - 180;

    float shortest_distance = clockwise_distance;
	
    if (counter_clockwise_distance < shortest_distance) {
        shortest_distance = -counter_clockwise_distance;
    }
	*reverflag = 1.0;
	if(fabsf(shortest_distance)>90){
		float flipped_yaw_now = yaw_now_360 + 180;
		
		if (flipped_yaw_now >= 360) {
			flipped_yaw_now -= 360;
		}
		if(clockwise_distance > counter_clockwise_distance)
		{
			reverse_distance = fmodf((yaw_set_360 - flipped_yaw_now + 360), 360);
		}else{
			reverse_distance = -fmodf(flipped_yaw_now - yaw_set_360 + 360, 360);
		}
		*reverflag = -1.0;
        shortest_distance = reverse_distance;
	}
    return shortest_distance;
}

float LowPass_SetChassis(float old,float In)
{
    return (1-K_Low)*(old)+K_Low*In;   
}
// ***************************************** End of Helping *****************************************//

// ***************************************** Begin of Init*********************************************//
void chassis_init(void) {
	init_delta_angle=CAN1_Motors[YAW_ID].totalAngle-YAW_OFFSET_FORWARD;
	chassic_angel = attitude.totalYaw; 
	chassis.info.gimbal_angle = chassic_angel;

	chassis.info.vx_set = 0.f;
	chassis.info.vy_set = 0.f;
	chassis.info.vw_set = 0.f;
	chassis.info.follow = FOLLOW_OFF;
	
	chassis.info.vx_get = 0.f;
	chassis.info.vy_get = 0.f;
	chassis.info.vw_get = 0.f;
	
	chassis.info.steeringwheel_offset[STEERING_RF_ID] = STEERING_RF_OFFSET;
	chassis.info.steeringwheel_offset[STEERING_LF_ID] = STEERING_LF_OFFSET;
	chassis.info.steeringwheel_offset[STEERING_LB_ID] = STEERING_LB_OFFSET;
	chassis.info.steeringwheel_offset[STEERING_RB_ID] = STEERING_RB_OFFSET;
	
	chassis.command.vx_set = 0.f;
	chassis.command.vy_set = 0.f;
	chassis.command.vw_set = 0.f;
	chassis.command.follow = FOLLOW_OFF;
	chassis.command.chassis_move.car_v = 0.0f;
	chassis.command.chassis_move.car_yaw = 0.0f;
	
	for (int i = 1; i <= 4; i ++){
		chassis.info.speed_get[Wheel_ID_list[i]] = 0.f;
		chassis.info.speed_set[Wheel_ID_list[i]] = 0.f;
		chassis.command.speed_set[Wheel_ID_list[Wheel_ID_list[i]]] = 0.f;
		chassis.command.steering_angel_set[Steering_ID_list[i]] = 0.f;
		chassis.command.steering_code_set[Steering_ID_list[i]] = 0.f;
		chassis.info.last_speed_set[Wheel_ID_list[i]] = 0.0f;
		chassis.info.last_steering_code_set[Steering_ID_list[i]] = 0.0f;
	}
	
	PID_init(&chassis.wheel_pid[CHA_RF_ID], WHEEL_RF_PID);
	PID_init(&chassis.wheel_pid[CHA_LF_ID], WHEEL_LF_PID);
	PID_init(&chassis.wheel_pid[CHA_LB_ID], WHEEL_LB_PID);
	PID_init(&chassis.wheel_pid[CHA_RB_ID], WHEEL_RB_PID);
	PID_init(&chassis.rotate, ROTATE_PID);
	
	FSFC_Init(&chassis.steering_fsfc[STEERING_RF_ID], STEERING_RF_FSFC);
	FSFC_Init(&chassis.steering_fsfc[STEERING_LF_ID], STEERING_LF_FSFC);
	FSFC_Init(&chassis.steering_fsfc[STEERING_LB_ID], STEERING_LB_FSFC);
	FSFC_Init(&chassis.steering_fsfc[STEERING_RB_ID], STEERING_RB_FSFC);
	FSFC_Init(&YAW_Dir,YAW_DIR);
	
	CanMotor_ResetTotalAngle(&CAN1_Motors[YAW_ID]);
	for(int i = 1; i<=4; i++){
		CanMotor_ResetTotalAngle(&CAN2_Motors[Steering_ID_list[i]]);
		chassis.command.steering_code_set[Steering_ID_list[i]] = CAN2_Motors[Steering_ID_list[i]].totalAngle;
	}
}
// ***************************************** End of Init*********************************************//

// ***************************************** Begin of Set*********************************************//
void chassis_set_command(float vx, float vy, float vw, chassis_follow_t follow) {
		for (int i = 1; i <= 4; i ++){
		chassis.info.last_speed_set[Wheel_ID_list[i]] = chassis.command.speed_set[Wheel_ID_list[i]];
		chassis.info.last_steering_code_set[Steering_ID_list[i]] = chassis.command.steering_code_set[Steering_ID_list[i]];
	}
		
	chassis.command.vx_set = vx;
	chassis.command.vy_set = vy;
	chassis.command.vw_set = vw;
	chassis.command.follow = follow;
	
	chassis.command.chassis_move.car_yaw = R2DEG(atan2f(chassis.command.vy_set, chassis.command.vx_set)) - chassis.info.gimbal_angle;
	
	chassis.command.chassis_move.car_v = sqrtf(chassis.command.vx_set * chassis.command.vx_set + chassis.command.vy_set * chassis.command.vy_set);
	
	for(int i = 1; i <= 4; i++){
		chassis.command.speed_set[Wheel_ID_list[i]] = chassis.command.chassis_move.car_v;
		chassis.command.steering_angel_set[Steering_ID_list[i]] = chassis.command.chassis_move.car_yaw;
	}
	
	if (vw != 0){
		float wheel_temp_v[5] = {0};
		float wheel_temp_yaw[5] = {0};
		wheel_temp_v[1] = -chassis.command.vw_set;
		wheel_temp_v[2] = +chassis.command.vw_set;
		wheel_temp_v[3] = +chassis.command.vw_set;
		wheel_temp_v[4] = -chassis.command.vw_set;
		
		wheel_temp_yaw[1] = -_CIRCLE_ANGEL;
		wheel_temp_yaw[2] = +_CIRCLE_ANGEL;
		wheel_temp_yaw[3] = -_CIRCLE_ANGEL;
		wheel_temp_yaw[4] = +_CIRCLE_ANGEL;
		
		for(int i = 1; i <= 4; i++){
			float vxi = 
				        chassis.command.speed_set[Wheel_ID_list[i]] * cosf(DEG2R(chassis.command.steering_angel_set[Steering_ID_list[i]])) + 
								wheel_temp_v[i] * cosf(DEG2R(wheel_temp_yaw[i]));
			float vyi = 
				        chassis.command.speed_set[Wheel_ID_list[i]] * sinf(DEG2R(chassis.command.steering_angel_set[Steering_ID_list[i]])) + 
								wheel_temp_v[i] * sinf(DEG2R(wheel_temp_yaw[i]));
			chassis.command.speed_set[Wheel_ID_list[i]] = sqrtf(vxi * vxi + vyi * vyi);
			chassis.command.steering_angel_set[Steering_ID_list[i]] = R2DEG(atan2f(vyi, vxi));
		}
	}
	
	float yaw_now_360=0,yaw_set_360=0,yaw_diff_360=0;
	for(int i = 1; i <= 4; i++){
		yaw_now_360 = (CAN2_Motors[Steering_ID_list[i]].angle - chassis.info.steeringwheel_offset[Steering_ID_list[i]]) / 8192.0f * 360.0f;
		if(yaw_now_360 < 0) yaw_now_360 += 360;
		yaw_set_360 = fmodf(chassis.command.steering_angel_set[Steering_ID_list[i]], 360.0f);
		if(yaw_set_360 < 0) yaw_set_360 += 360;
		float reverse_flag = 1.0f;
		yaw_diff_360 = calculateShortestDistance(yaw_now_360, yaw_set_360, &reverse_flag);
		chassis.command.steering_angel_set[Steering_ID_list[i]] = chassis.command.steering_angel_set[Steering_ID_list[i]] + yaw_diff_360;
		chassis.command.steering_code_set[Steering_ID_list[i]] = CAN2_Motors[Steering_ID_list[i]].totalAngle+DEG2CODE(yaw_diff_360);
		chassis.command.speed_set[Wheel_ID_list[i]] *=reverse_flag;
		float cos_k = cosf(DEG2R(yaw_diff_360));
		chassis.command.speed_set[Wheel_ID_list[i]] *= fabs(cos_k * cos_k * cos_k);
	}
	chassis.command.speed_set[CHA_RF_ID] *= -1;
	chassis.command.speed_set[CHA_RB_ID] *= -1;
	//to different mortor there maybe need to apply the dir of the mortor (+-1)
	for(uint8_t i = 1; i<=4 ; i++){
		LowPass_SetChassis(chassis.info.last_speed_set[Wheel_ID_list[i]], chassis.command.speed_set[Wheel_ID_list[i]]);
		LowPass_SetChassis(chassis.info.last_steering_code_set[Steering_ID_list[i]], chassis.command.steering_code_set[Steering_ID_list[i]]);
	}
}
// ***************************************** End of Set*********************************************//

void chassis_info_update(void) {
	if(main_control_mode.control_mode==CONTROLLER_MODE){
		YAW_ANGLE = chassisAngle();
		chassic_angel = attitude.totalYaw;
		gimbal_angel = CAN1_Motors[YAW_ID].totalAngle;
		chassis.info.gimbal_angle = attitude.totalYaw - chassic_angel;
		if(chassis.info.gimbal_angle>180) chassis.info.gimbal_angle-=360;
		if(chassis.info.gimbal_angle<-180) chassis.info.gimbal_angle+=360;
	} 
	
	if(main_control_mode.control_mode==MOUSE_KEY_MODE){
		chassic_angel -= 2*control.channel[2];
		YAW_ANGLE = gimbal_Angle(gimbal_angel);
		chassis.info.gimbal_angle = chassic_angel- attitude.totalYaw;
	} 

	float real_spd[5];
	chassis.info.speed_get[CHA_RF_ID] = CAN1_Motors[CHA_RF_ID].speed;
	chassis.info.speed_get[CHA_LF_ID] = CAN1_Motors[CHA_LF_ID].speed;
	chassis.info.speed_get[CHA_LB_ID] = CAN1_Motors[CHA_LB_ID].speed;
	chassis.info.speed_get[CHA_RB_ID] = CAN1_Motors[CHA_RB_ID].speed;
    
	chassis.info.speed_set[CHA_RF_ID] = chassis.command.speed_set[CHA_RF_ID];
	chassis.info.speed_set[CHA_LF_ID] = chassis.command.speed_set[CHA_LF_ID];
	chassis.info.speed_set[CHA_LB_ID] = chassis.command.speed_set[CHA_LB_ID];
	chassis.info.speed_set[CHA_RB_ID] = chassis.command.speed_set[CHA_RB_ID];
    
	real_spd[CHA_RF_ID] = CAN1_Motors[CHA_RF_ID].speed / M3508_DECELE_RATIO / 60.0f * 2 * PI;
	real_spd[CHA_LF_ID] = CAN1_Motors[CHA_LF_ID].speed / M3508_DECELE_RATIO / 60.0f * 2 * PI;
	real_spd[CHA_LB_ID] = CAN1_Motors[CHA_LB_ID].speed / M3508_DECELE_RATIO / 60.0f * 2 * PI;
	real_spd[CHA_RB_ID] = CAN1_Motors[CHA_RB_ID].speed / M3508_DECELE_RATIO / 60.0f * 2 * PI;
	
	#if CHASSIS_TYPE == OMNI
	
	chassis.info.vx_get = SQUARE_ROOT_OF_2 * (+ real_spd[CHA_RF_ID] + real_spd[CHA_LF_ID] - real_spd[CHA_LB_ID] - real_spd[CHA_RB_ID]) * _WHEEL_RADIUS / 4;
	chassis.info.vy_get = SQUARE_ROOT_OF_2 * (+ real_spd[CHA_RF_ID] - real_spd[CHA_LF_ID] - real_spd[CHA_LB_ID] + real_spd[CHA_RB_ID]) * _WHEEL_RADIUS / 4;
	chassis.info.vw_get = 1 / _CHASSIS_RADIUS * (real_spd[CHA_RF_ID] + real_spd[CHA_LF_ID] + real_spd[CHA_LB_ID] + real_spd[CHA_RB_ID]) * _WHEEL_RADIUS / 4;
	
	#elif CHASSIS_TYPE == MECANUM
	
	chassis.info.vx_get = (+ real_spd[CHA_RF_ID] + real_spd[CHA_LF_ID] - real_spd[CHA_LB_ID] - real_spd[CHA_RB_ID]) * _WHEEL_RADIUS / 4;
	chassis.info.vy_get = (+ real_spd[CHA_RF_ID] - real_spd[CHA_LF_ID] - real_spd[CHA_LB_ID] + real_spd[CHA_RB_ID]) * _WHEEL_RADIUS / 4;
	chassis.info.vw_get = 1 / (_CHASSIS_WHEELBASE+_CHASSIS_WHEELTRACK) * (real_spd[CHA_RF_ID] + real_spd[CHA_LF_ID] + real_spd[CHA_LB_ID] + real_spd[CHA_RB_ID]) * _WHEEL_RADIUS / 4;
	
	#endif
	
	chassis.info.vx_set = (+ cos((chassis.info.gimbal_angle/180)*PI) * (chassis.command.vx_set) - sin((chassis.info.gimbal_angle/180)*PI) * (chassis.command.vy_set));
	chassis.info.vy_set = (+ sin((chassis.info.gimbal_angle/180)*PI) * (chassis.command.vx_set) + cos((chassis.info.gimbal_angle/180)*PI) * (chassis.command.vy_set));	
	chassis.info.vw_set = chassis.command.vw_set;
	chassis.info.follow = chassis.command.follow;
}
void chassis_info_get(chassis_info_t *info){
	memcpy(info, &chassis.info, sizeof(chassis_info_t));
}
void chassis_excute(void) {
	chassis_info_t *info = &chassis.info;
	chassis_command_t *cmd = &chassis.command;
	if (!control.online || main_control_mode.control_mode == OFF_MODE){
		for (int i = 1; i <= 8; i ++){
			CAN1_MotorCurrents[i] = 0;
		}
		for (int i = 5; i <= 8; i ++){
			CAN2_MotorCurrents[i] = 0;
		}
	}
	else{
		#if POWER_CONTROL_ON
			for (uint8_t i = 1; i <= 4; i++) {
				power.current_set[Wheel_ID_list[i]] = PID_calc(&chassis.wheel_pid[Wheel_ID_list[i]],
				info->speed_get[Wheel_ID_list[i]], cmd->speed_set[Wheel_ID_list[i]]);
			}
			power.cmd.sum_abs_w = 0.0f;
			power.cmd.sum_tau2  = 0.0f;
			power.cmd.sum_tau_w = 0.0f;
			for (uint8_t i = 1; i <= 4; i++) {
				cmd_motor_info_t *motor = &power.cmd.motor[Wheel_ID_list[i]];
				
				motor->speed  = info->speed_get[Wheel_ID_list[i]] * RPM_TO_RADPS;													// [rad/s] 	motor shaft, before gearbox
				motor->torque = power.current_set[Wheel_ID_list[i]] * CURRENT_TO_AMPERE * TORQUE_CONSTANT;	// [N*m] 		motor shaft, before gearbox
				motor->err    = (cmd->speed_set[Wheel_ID_list[i]] - info->speed_get[Wheel_ID_list[i]]) * RPM_TO_RADPS;		// [rad/s]
				
				float tau_w = motor->torque * motor->speed;
				float abs_w = fabsf(motor->speed);
				float tau2  = motor->torque * motor->torque;
				
				motor->P_cmd = tau_w + power.info.k1*abs_w + power.info.k2*tau2 + power.info.k3*0.25f; // [W]
				
				power.cmd.sum_abs_w += abs_w;
				power.cmd.sum_tau2  += tau2;
				power.cmd.sum_tau_w += tau_w;
			}
			power_control();
			for (uint8_t i = 1; i <= 4; i++) {
				CAN1_MotorCurrents[Wheel_ID_list[i]] = power.current_set[Wheel_ID_list[i]];
			}
			CAN2_MotorCurrents[STEERING_RF_ID] = FSFC_calc(&chassis.steering_fsfc[STEERING_RF_ID], CAN2_Motors[STEERING_RF_ID].totalAngle, cmd->steering_code_set[STEERING_RF_ID], CAN2_Motors[STEERING_RF_ID].speed, 0);
			CAN2_MotorCurrents[STEERING_LF_ID] = FSFC_calc(&chassis.steering_fsfc[STEERING_LF_ID], CAN2_Motors[STEERING_LF_ID].totalAngle, cmd->steering_code_set[STEERING_LF_ID], CAN2_Motors[STEERING_LF_ID].speed, 0);
			CAN2_MotorCurrents[STEERING_LB_ID] = FSFC_calc(&chassis.steering_fsfc[STEERING_LB_ID], CAN2_Motors[STEERING_LB_ID].totalAngle, cmd->steering_code_set[STEERING_LB_ID], CAN2_Motors[STEERING_LB_ID].speed, 0);
			CAN2_MotorCurrents[STEERING_RB_ID] = FSFC_calc(&chassis.steering_fsfc[STEERING_RB_ID], CAN2_Motors[STEERING_RB_ID].totalAngle, cmd->steering_code_set[STEERING_RB_ID], CAN2_Motors[STEERING_RB_ID].speed, 0);
			CAN1_MotorCurrents[YAW_ID] = FSFC_calc(&YAW_Dir,YAW_ANGLE,info->gimbal_angle,CAN1_Motors[YAW_ID].speed,0);
		#else
			CAN1_MotorCurrents[CHA_LB_ID] = PID_calc(&chassis.wheel_pid[CHA_LB_ID] , info->speed_get[CHA_LB_ID] , info->speed_set[CHA_LB_ID]);
			CAN1_MotorCurrents[CHA_RB_ID] = PID_calc(&chassis.wheel_pid[CHA_RB_ID] , info->speed_get[CHA_RB_ID] , info->speed_set[CHA_RB_ID]);
			CAN1_MotorCurrents[CHA_LF_ID] = PID_calc(&chassis.wheel_pid[CHA_LF_ID] , info->speed_get[CHA_LF_ID] , info->speed_set[CHA_LF_ID]);
			CAN1_MotorCurrents[CHA_RF_ID] = PID_calc(&chassis.wheel_pid[CHA_RF_ID] , info->speed_get[CHA_RF_ID] , info->speed_set[CHA_RF_ID]);
			for(int i = 1; i <= 4; i++){
				VAL_LIMIT(CAN1_MotorCurrents[Wheel_ID_list[i]], -15000, 15000);
			}
			CAN2_MotorCurrents[STEERING_RF_ID] = FSFC_calc(&chassis.steering_fsfc[STEERING_RF_ID], CAN2_Motors[STEERING_RF_ID].totalAngle, cmd->steering_code_set[STEERING_RF_ID], CAN2_Motors[STEERING_RF_ID].speed, 0);
			CAN2_MotorCurrents[STEERING_LF_ID] = FSFC_calc(&chassis.steering_fsfc[STEERING_LF_ID], CAN2_Motors[STEERING_LF_ID].totalAngle, cmd->steering_code_set[STEERING_LF_ID], CAN2_Motors[STEERING_LF_ID].speed, 0);
			CAN2_MotorCurrents[STEERING_LB_ID] = FSFC_calc(&chassis.steering_fsfc[STEERING_LB_ID], CAN2_Motors[STEERING_LB_ID].totalAngle, cmd->steering_code_set[STEERING_LB_ID], CAN2_Motors[STEERING_LB_ID].speed, 0);
			CAN2_MotorCurrents[STEERING_RB_ID] = FSFC_calc(&chassis.steering_fsfc[STEERING_RB_ID], CAN2_Motors[STEERING_RB_ID].totalAngle, cmd->steering_code_set[STEERING_RB_ID], CAN2_Motors[STEERING_RB_ID].speed, 0);
			CAN1_MotorCurrents[YAW_ID] = FSFC_calc(&YAW_Dir,YAW_ANGLE,info->gimbal_angle,CAN1_Motors[YAW_ID].speed,0);
		#endif
	}
	CanMotor_SendCurrent(CAN1_MOTOR_1_TO_4);
	CanMotor_SendCurrent(CAN1_MOTOR_5_TO_8);
	CanMotor_SendCurrent(CAN2_MOTOR_5_TO_8);
}

void chassis_off(void) {
	for (int i = 1; i <= 8; i ++){
			CAN1_MotorCurrents[i] = 0;
		}
		for (int i = 5; i <= 8; i ++){
			CAN2_MotorCurrents[i] = 0;
		}
	CanMotor_SendCurrent(CAN1_MOTOR_1_TO_4);
	CanMotor_SendCurrent(CAN1_MOTOR_5_TO_8);
	CanMotor_SendCurrent(CAN2_MOTOR_5_TO_8);
}
// ***************************************** Begin of Chassis_task*********************************************//
void chassis_task(void *args) {
	chassis_init();
//	power_control_init();
	thread_status.chassis = THREAD_OK;
	while(!(thread_status.uartcom  && //waiting till all task threads are initialized
          thread_status.cancom   &&    
          //thread_status.ui       &&
          thread_status.chassis  &&
          //thread_status.gimbal   &&
          //thread_status.shoot    &&
          thread_status.mainctrl )){
		delay(50);
	}
    
	while (1) {
		#if CHASSIS_ON
		#if POWER_CONTROL_ON
		power_control_info_update();
		#endif
		chassis_info_update();
		chassis_excute();
		#else
		chassis_off();
		#endif
		delay(5);
	}
}
// ***************************************** End of Chassis_task*********************************************//
