#define _GIMBAL_C

#include "gimbal.h"
#include "comm.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern thread_status_t thread_status;

// new module
gimbal_t gimbal;


// ****************************************** Begin of Init ******************************************//

void gimbal_init(void) {
	gimbal.command.yaw_angle_set = 0.f;
	gimbal.command.yaw_speed_set = 0.f;
	gimbal.command.pitch_angle_set = 0.f;
	gimbal.command.pitch_speed_set = 0.f;
	
	gimbal.info.yaw_angle_get = 0.f;
	gimbal.info.yaw_speed_get = 0.f;
	gimbal.info.pitch_angle_get = 0.f;
	gimbal.info.pitch_speed_get = 0.f;
	
	FSFC_Init(&gimbal.yaw_fsfc, YAW_FSFC_PARAMS);
	FSFC_Init(&gimbal.pitch_fsfc, PITCH_FSFC_PARAMS);
}

// ****************************************** End of Init ******************************************//

// **************************************** Begin of Interface Function *****************************************//

void gimbal_set_command(float yaw_angle, float yaw_speed, float pitch_angle, float pitch_speed) {
	gimbal.command.yaw_angle_set = yaw_angle;
	gimbal.command.yaw_speed_set = yaw_speed;
	gimbal.command.pitch_angle_set = pitch_angle;
	gimbal.command.pitch_speed_set = pitch_speed;

	VAL_LIMIT(gimbal.command.pitch_angle_set, gimbal.info.pitch_lower_bound, gimbal.info.pitch_upper_bound);
	
	#if LIMIT_YAW
	VAL_LIMIT(gimbal.command.yaw_angle_set, gimbal.info.yaw_lower_bound, gimbal.info.yaw_upper_bound);
	#endif
}

void gimbal_info_get(gimbal_info_t *info) {
	memcpy(info, &gimbal.info, sizeof(gimbal_info_t));
}

// **************************************** End of Interface Function *****************************************//

// **************************************** Begin of Helper Function ****************************************//

void gimbal_execute(void) {
	if (!control.online){
		CAN2_MotorCurrents[YAW_ID] = 0;
		CAN2_MotorCurrents[PITCH_ID] = 0;
	}
	else {
		CAN2_MotorCurrents[YAW_ID] = YAW_MOTOR_DIR * FSFC_calc(&gimbal.yaw_fsfc,
			gimbal.info.yaw_angle_get, gimbal.command.yaw_angle_set, gimbal.info.yaw_speed_get, gimbal.command.yaw_speed_set);
		CAN1_MotorCurrents[PITCH_ID] = PITCH_MOTOR_DIR * FSFC_calc(&gimbal.pitch_fsfc,
			gimbal.info.pitch_angle_get, gimbal.command.pitch_angle_set, gimbal.info.pitch_speed_get, gimbal.command.pitch_speed_set);
	}
	
	CanMotor_SendCurrent(CAN1_MOTOR_5_TO_8);
}

void get_std_angle(float *yaw, float *pitch, float *roll, float *yaw_spd, float *pitch_spd, float *roll_spd){
	*yaw = -attitude.totalYaw;
	*pitch = attitude.pitch;
	*roll = -attitude.roll;
	
	*yaw_spd = -attitude.gyroYaw;
	*pitch_spd = attitude.gyroPitch;
	*roll_spd = -attitude.gyroRoll;
}

void gimbal_info_update(void) {
	float yaw, pitch, roll;
	float yaw_speed, pitch_speed, roll_speed;
	get_std_angle(&yaw, &pitch, &roll, &yaw_speed, &pitch_speed, &roll_speed);
	
	gimbal.info.yaw_angle_set = gimbal.command.yaw_angle_set;
	gimbal.info.yaw_speed_set = gimbal.command.yaw_speed_set;
	gimbal.info.pitch_angle_set = gimbal.command.pitch_angle_set;
	gimbal.info.pitch_speed_set = gimbal.command.pitch_speed_set;
	
	gimbal.info.yaw_angle_get = yaw;
	gimbal.info.yaw_speed_get = yaw_speed;
	gimbal.info.pitch_angle_get = pitch * DEGREE_TO_MOTOR;
	gimbal.info.pitch_speed_get = pitch_speed;
	
	gimbal.info.yaw_motor_angle_get = CAN2_Motors[YAW_ID].angle;
	
	#if LIMIT_YAW
	gimbal.info.yaw_upper_bound = yaw + (gimbal.info.yaw_motor_angle_get - YAW_MOTOR_LOWER_BOUND)/DEGREE_TO_MOTOR;
	gimbal.info.yaw_lower_bound = yaw + (gimbal.info.yaw_motor_angle_get - YAW_MOTOR_UPPER_BOUND)/DEGREE_TO_MOTOR;
	#endif
	
	gimbal.info.pitch_lower_bound = PITCH_IMU_LOWER_BOUND;
	gimbal.info.pitch_upper_bound = PITCH_IMU_UPPER_BOUND;
}

// **************************************** End of Helper Function ****************************************//

void gimbal_task(void *args) {
	gimbal_init();
    
    thread_status.gimbal = THREAD_OK;
    while(!(thread_status.uartcom  && //waiting till all task threads are initialized
            thread_status.cancom   &&    
            thread_status.ui       &&
            thread_status.chassis  &&
            //thread_status.gimbal   &&
            //thread_status.shoot    &&
            thread_status.mainctrl )){
        delay(50);
    }
            
	while(1) {
		gimbal_info_update();
		gimbal_execute();
		delay(5);
	}
}
