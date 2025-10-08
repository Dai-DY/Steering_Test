#ifndef _ROBOT_CONF_H_
#define _ROBOT_CONF_H_


// some special mode
#define CV_DEBUG 0

// for adjustment
#define CV_PID_ADJUST 0
// for new function test


// ******** begin macro of motor ID ******** //

#define CHA_RF_ID 2
#define CHA_LF_ID 1
#define CHA_LB_ID 4
#define CHA_RB_ID 3

#define STEERING_RF_ID 5
#define STEERING_LF_ID 8
#define STEERING_LB_ID 7
#define STEERING_RB_ID 6
#define STEERING_INDEX_DELTA 5

#define FRIC1_ID 2 // left friction wheel
#define FRIC2_ID 1 // right friction wheel
#define PRWHEEL_ID 3 // puller wheel
#define PITCH_ID 6
#define YAW_ID 5

// ******** end macro of motor ID ******** //


// ******** begin macro of chassis ******** //

#define CHASSIS_ON 1

#define MECANUM 1
#define OMNI 0

#define ACC_ALLOWED 10.0f // acceleration limit of chassis
#define DEC_ALLOWED 10.0f // deceleration limit of chassis

#define YAW_OFFSET_FORWARD 2875 // 0-8192, yaw value when gimbal is directed at armor 0, which is the positive forward direction of chassis
#define STEERING_RF_OFFSET 3406
#define STEERING_LF_OFFSET 124
#define STEERING_LB_OFFSET 6150
#define STEERING_RB_OFFSET 1440

/* chassis maximum translation speed, unit is mm/s */
#define _CHASSIS_MAX_VX_SPEED 1500.0f
#define _CHASSIS_MAX_VY_SPEED 1500.0f
/* chassis maximum rotation speed, unit is degree/s */
#define _CHASSIS_MAX_VR_SPEED 920.0f

// PID of wheels
#define WHEEL_RF_PID 18, 0, 0.2, 1500, 250, 8000, 0, 1
#define WHEEL_LF_PID 18, 0, 0.2, 1500, 250, 8000, 0, 1
#define WHEEL_LB_PID 18, 0, 0.2, 1500, 250, 8000, 0, 1
#define WHEEL_RB_PID 18, 0, 0.2, 1500, 250, 8000, 0, 1

#define STEERING_RF_FSFC 2500, 0, 0, 4000, 8000, 1
#define STEERING_LF_FSFC 2500, 0, 0, 4000, 8000, 1
#define STEERING_LB_FSFC 2500, 0, 0, 4000, 8000, 1
#define STEERING_RB_FSFC 2500, 0, 0, 4000, 8000, 1
#define ROTATE_PID 		3, 0, 10, 1000, 1200, 200, 0, 1 // PID of following

// ******** end macro of chassis ******** //


// ******** begin macro of gimbal ******** //

#define GIMBAL_ON 0

#define LIMIT_YAW 0	// if on, then the yaw is contraint by a range of motor angle
#define YAW_MOTOR_LOWER_BOUND 2000
#define YAW_MOTOR_UPPER_BOUND 4000

#define YAW_MOTOR_DIR -1		// the relative direction of motor and CV yaw axis
#define PITCH_MOTOR_DIR -1	// the relative direction of motor and CV pitch axis

#define PITCH_IMU_UPPER_BOUND 14*DEGREE_TO_MOTOR // maximum pitch angle, unit: motor
#define PITCH_IMU_LOWER_BOUND -14*DEGREE_TO_MOTOR // minimum pitch angle, unit: motor

#define PITCH_UPWARD_RANGE 26 // pitch angle range of upward (always positive), unit: deg
#define PITCH_DOWNWARD_RANGE -14 // pitch angle range of downward (always negative), unit: deg
#define PITCH_OFFSET 4296 // motor value when pitch angle equals zero


#define YAW_FSFC_PARAMS		6000.0f, 11000.0f, 0.0f, 4000, 30000, 1
#define PITCH_FSFC_PARAMS		170.0f, 10000.0f, 0.0f, 4000, 30000, 1

// ******** end macro of gimbal ******** //


// ******** begin macro of shoot ******** //

#define SHOOT_ON 1 // whether shoot component is used

#define LOCAL_HEAT_MANAGEMENT_ON 0 // whether local heat management is used, caution: only when judgement system is connected, local heat management is correct

#define FRIC_DIR -1 // 1 if left friction wheel's positive direction equals rotation direction when shooting
#define PRWHEEL_DIR -1 // 1 if puller wheel's positive direction equals rotation direction when shooting

// for checking shoot block
#define BLOCK_TIME_THRESHOLD 70 // threshold of block time to determine that prwheel is blocked

// for count_fired_bullet to determine that a bullet is fired
#define FRIC_SPEED_DECREASE_INTEGRAL_THRESHOLD -6 // threshold of continuous decrease of fric speed
#define FRIC_SPEED_DECREASE_THRESHOLD 9 // threshold of decrease of fric speed relative to fric speed target
#define DIFFERENTIAL_THRESHOLD 0.1f // threshold of change of fric speed which is counted into fric_speed_decrease_integral

// for prwheel back set
#define BACK_SET_CURRENT 2000 // motor current for prwheel back set
#define BACK_SET_TIME  200 // the duation of prwheel back set

#define BULLET_OVERSPEED_THRESHOLD 15.8f // threshold of overspeed

#define VALID_SHOOT_SPEED_THRESHOLD 18.0f // threshold of valid shoot speed measured by judgement system

#define AVERAGE_SHOOT_SPEED 24.0f 

#define HEAT_PER_SHOOT 10 
#define REMAINING_HEAT 30 // when you remain such amount of heat, you are not allowed to shoot, used for local heat management

#define FRIC_LEFT_PID 	20, 0, 0, 500, 2000, 14000, 0, 1
#define FRIC_RIGHT_PID	20, 0, 0, 500, 2000, 14000, 0, 1
#define PREWHELL_PID		120, 0.03, 27, 1000, 2000, 10000, 0, 1

// ******** end macro of shoot ******** //


// ******** begin macro of constants ******** //

#define PI 3.1416f
#define DEGREE_TO_MOTOR  22.7556f // 8192/360
#define RADIAN_TO_DEGREE 57.3f 		// 180/3.14
#define DEGREE_TO_RADIAN 0.0174f 	// 3.14f/180.0f
#define SQUARE_ROOT_OF_2 1.41421356f
#define SQUARE_ROOT_OF_2_DIVIDED_BY_2 0.70710678f

#define M3508_MAX_WHEEL_RPM 8740 // rotor speed
#define M3508_DECELE_RATIO 19.0f // M3508_decelerate_ratio
#define M2006_REDUCTION_RATIO 36.0f // M2006_decelerate_ratio
#define GM6020_MAX_CODE_DISC_VALUE 8192
#define GM6020_CODE_DISC_VALUE_CHANGE_PER_90_DEGREE 2048

// ******** end macro of constants ******** //


// ******** begin macro of robot Mechanical constants ******** //

#define _WHEEL_RADIUS      72.0f  // the radius of wheel(mm)
#define _WHEEL_PERIMETER   358.0f // the perimeter of wheel(mm)

#define _CHASSIS_WHEELTRACK  380.0f // wheel track distance(mm)
#define _CHASSIS_WHEELBASE   230.0f // wheel base distance(mm)
#define _CHASSIS_RADIUS      480.0f // radius of chassis(mm)
#define _CIRCLE_ANGEL        45     // arctan(a/b) a = length b = width

// ******** end macro of robot Mechanical constants ******** //

#include "comm.h"

static int Steering_ID_list[5] = {0 , STEERING_RF_ID , STEERING_LF_ID , STEERING_LB_ID , STEERING_RB_ID};
static int Wheel_ID_list[5]  = {0, CHA_RF_ID, CHA_LF_ID, CHA_LB_ID, CHA_RB_ID};

#endif
