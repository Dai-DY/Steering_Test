#ifndef _COMPONENTS_DM_MOTOR_H
#define _COMPONENTS_DM_MOTOR_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define RAD_TO_DEGREE 180.f / 3.14159265f
#define DEGREE_TO_RAD 3.14159265f / 180.f

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

typedef enum{
	USE_CAN1=1,
	USE_CAN2=2,
}DM_Motor_CAN_t;

typedef enum{
	Disabled=0,
	Enabled=1,
	Initing=2,
	OverVoltage=8,
	UnderVoltage=9,
	OverCurrent=0xA,
	OverMosTemp=0xB,
	OverCoilTemp=0xC,
	CommFail=0xD,
	OverLoad=0xE,
	ParamErr=0xF,
}DM_Motor_status_t;

typedef struct{
	uint16_t id;
	uint8_t mode;
	DM_Motor_status_t state;
	float lastpos;
	float totalpos;
	float pos;
	float vel;
	float tor;
	float Tmos;
	float Tcoil;
	float P_max;
	float V_max;
	float T_max;
	int32_t refPosCycle;
	uint32_t transmit_time;
	uint32_t receive_time;
}DM_Motor_t;

extern DM_Motor_t DM_Motors_CAN1[];
extern DM_Motor_t DM_Motors_CAN2[];

void DMMotor_General_Init(void);

HAL_StatusTypeDef DMMotor_Init(DM_Motor_CAN_t which_can, uint16_t id);
HAL_StatusTypeDef DMMotor_DeInit(DM_Motor_CAN_t which_can, uint16_t id);

void DMMotor_ESCDataUnpack(const uint8_t* data, DM_Motor_t* motors);
HAL_StatusTypeDef DMMotor_MIT_SendCommand(DM_Motor_CAN_t which_can, uint16_t id, float position, float velocity, float torque, float Kp, float Kd);

#endif

