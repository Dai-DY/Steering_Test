#include "zero_check.h"
#include "chassis.h"

ZeroCheck_Typedef ZeroCheck_Yaw_Steering[9];
float ZeroCheck(ZeroCheck_Typedef *Zero,float value,short Zerocheck_mode)
{
	Zero->ActualValue=value;
	Zero->PreError=Zero->ActualValue-Zero->LastValue;
	Zero->LastValue=Zero->ActualValue;
	
	if(Zero->PreError>0.7f*Zero->CountCycle)
	{
		Zero->PreError=Zero->PreError-Zero->CountCycle;
		Zero->Circle++;
	}
	if(Zero->PreError<-0.7f*Zero->CountCycle)
	{
		Zero->PreError=Zero->PreError+Zero->CountCycle;
		Zero->Circle--;
	}
	return Zero->ActualValue - Zero->Circle*Zero->CountCycle;
}

void ZeroCheck_cal(void)
{
	for (int i=1;i<=4;i++)
	{
		float decode_angle = ZeroCheck(&(ZeroCheck_Yaw_Steering[Steering_ID_list[i]]),CAN2_Motors[Steering_ID_list[i]].angle,Position);
		chassis.info.steering_decode_trans_angle[Steering_ID_list[i]] = (decode_angle - chassis.info.steeringwheel_offset[Steering_ID_list[i]])/8192.0f*360.0f;
	}	

}

void ZeroCheck_Init(void)
{
	for (int i=1;i<=4;i++)
	{
		ZeroCheck_Yaw_Steering[Steering_ID_list[i]].CountCycle = 8192;
		ZeroCheck_Yaw_Steering[Steering_ID_list[i]].LastValue = CAN2_Motors[Steering_ID_list[i]].angle;
		ZeroCheck_Yaw_Steering[Steering_ID_list[i]].Circle = 0;      
    ZeroCheck_Yaw_Steering[Steering_ID_list[i]].PreError = 0.0f; 
	}	
}
