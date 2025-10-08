#include "components.h"


void Components_Init()
{
	LED_Init();
	Controller_Init();
	IMU_Init();
	CanMotor_Init();
	DMMotor_General_Init();
	PWM_Init();
	USB_Init();
	Laser_5V_Enable();
}
