#ifndef _COMPONENTS_MOTOR_H
#define _COMPONENTS_MOTOR_H

#include <stdint.h>
#define MOTOR_ENCODER_RESOLUTION 8192
#define ESC_BASE_ID 0x200
#define CURRENTS_1_TO_4  0x200
#define CURRENTS_5_TO_8  0x1ff
#define CURRENTS_9_TO_12 0x2ff

#define WULIE_CAPACITY  1

typedef struct {
  int32_t   totalAngle;
  uint16_t  lastAngle;    // [0,8191] last rotor angle
  uint16_t  angle;        // [0,8191] now rotor angle
  int16_t   speed;        // rotor speed in RPM
  int16_t   current;      // [-16384,16384] for -20A to 20A current
  int8_t    temperature;  // celsius degree
} CanMotor;

typedef struct {
  int32_t   totalAngle;
  uint16_t  lastAngle;          // [0,8191] last rotor angle
  uint16_t  angle;              // [0,8191] now rotor angle
  int16_t   measuredCurrent;
  int16_t   targetCurrent;
} CanMotor_6623;

typedef enum  {
  CAN1_MOTOR_1_TO_4  = 0x0000U,
  CAN1_MOTOR_5_TO_8  = 0x0001U,
  CAN1_MOTOR_9_TO_12 = 0x0002U,
  CAN2_MOTOR_1_TO_4  = 0x0003U,
  CAN2_MOTOR_5_TO_8  = 0x0004U,
  CAN2_MOTOR_9_TO_12 = 0x0005U
} CAN_Motor_Selection;

extern CanMotor CAN1_Motors[];
extern CanMotor CAN2_Motors[];
extern int16_t CAN1_MotorCurrents[];
extern int16_t CAN2_MotorCurrents[];
extern float V_Cap;
extern float P_Cha;

void CanMotor_Init(void);
void CanMotor_SendCurrent(CAN_Motor_Selection selection);
void CanMotor_ESCDataUnpack(uint8_t const *data, CanMotor *motor);

#if WULIE_CAPACITY
//static inline void CAP_Unpack(const uint8_t * packet){
//	uint16_t * buf = (uint16_t *) packet;
//	int16_t * buf_ = (int16_t *)packet;
//	P_Cha = (float)buf_[0]/10.0f;
//	V_Cap = (uint16_t)buf[1]/100.0f;
//}
#else
static inline void CAP_Unpack(const uint8_t * packet){
  uint8_t* buf = (uint8_t*) &V_Cap;
	buf[0]=packet[0];
	buf[1]=packet[1];
	buf[2]=packet[2];
	buf[3]=packet[3];
}
#endif


void SendCapData(void);
void CAP_Unpack(const uint8_t * packet);

static inline void CanMotor_ResetTotalAngle(CanMotor *motor) {
  motor->totalAngle = 0;
}

#endif
