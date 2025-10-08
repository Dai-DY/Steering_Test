#include "dm_motor.h"
#include "os.h"
#include "string.h"

extern CAN_HandleTypeDef hcan1, hcan2;

DM_Motor_t DM_Motors_CAN1[16];
DM_Motor_t DM_Motors_CAN2[16];

#define DM_CLEARERR_FLAG 0xFB
#define DM_ENABLE_FLAG 0xFC
#define DM_DISABLE_FLAG 0xFD
#define DM_ZEROPOS_FLAG 0xFE

// Helper Function
float Hex_To_Float(uint32_t *Byte)
{
	return *((float*)Byte);
}

uint32_t FloatTohex(float HEX)
{
	return *( uint32_t *)&HEX;
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

HAL_StatusTypeDef dm_motor_magic(uint16_t id, uint8_t flag, CAN_HandleTypeDef* hcan){
	static uint8_t DM_Magic_Cmd[8]	= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0};
	static uint32_t mailbox;
	CAN_TxHeaderTypeDef header = {
		.IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 0x08,
		.StdId = id, .ExtId	= 0, .TransmitGlobalTime = DISABLE
	};
	DM_Magic_Cmd[7] = flag;
	return HAL_CAN_AddTxMessage(hcan, &header, DM_Magic_Cmd, &mailbox);
}

HAL_StatusTypeDef read_motor_data(uint16_t id, uint8_t rid, CAN_HandleTypeDef* hcan) 
{
	static uint32_t mailbox;
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
	CAN_TxHeaderTypeDef header = {
		.IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 0x04,
		.StdId = 0x7FF, .ExtId	= 0, .TransmitGlobalTime = DISABLE
	};
	return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

HAL_StatusTypeDef write_motor_data(uint16_t id, uint8_t rid, uint32_t value, CAN_HandleTypeDef* hcan)
{
	static uint32_t mailbox;
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, 0xff, 0xff, 0xff, 0xff};
	*((uint32_t*)(&data[4])) = value;
	CAN_TxHeaderTypeDef header = {
		.IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 0x08,
		.StdId = 0x7FF, .ExtId	= 0, .TransmitGlobalTime = DISABLE
	};
	return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

HAL_StatusTypeDef DMMotor_Init(DM_Motor_CAN_t which_can, uint16_t id)
{
	HAL_StatusTypeDef status;
	CAN_HandleTypeDef *hcan;
	DM_Motor_t *motor;
	if (which_can == USE_CAN1)
	{
		hcan = &hcan1;
		motor = &DM_Motors_CAN1[id];
	}
	else if (which_can == USE_CAN2)
	{
		hcan = &hcan2;
		motor = &DM_Motors_CAN2[id];
	}
	else
		return HAL_ERROR;
	if (id == 0 || id > 16)
	{
		while(1);
		// What Fucking Thing are you doing
		return HAL_ERROR;
	}
	status = dm_motor_magic(id, DM_CLEARERR_FLAG, hcan);
	if (status != HAL_OK)
	{
		return status;
	}
	motor->state = Disabled;
	osDelay(5);
	while(motor->state == Disabled && status == HAL_OK)
	{
		status = dm_motor_magic(id, DM_ENABLE_FLAG, hcan);
		osDelay(5);
	}
	if (status != HAL_OK || motor->state != Enabled)
	{
		return HAL_ERROR;
	}
	motor->state = Initing;
	motor->P_max = 0;
	motor->V_max = 0;
	motor->T_max = 0;
	motor->mode = 0;
	while(motor->P_max == 0 && status == HAL_OK)
	{
		status = read_motor_data(id, 0x15, hcan);
		osDelay(5);
	}
	if (status != HAL_OK)
	{
		return status;
	}
	while(motor->V_max == 0 && status == HAL_OK)
	{
		status = read_motor_data(id, 0x16, hcan);
		osDelay(5);
	}
	if (status != HAL_OK)
	{
		return status;
	}
	while(motor->T_max == 0 && status == HAL_OK)
	{
		status = read_motor_data(id, 0x17, hcan);
		osDelay(5);
	}
	if (status != HAL_OK)
	{
		return status;
	}
	while(motor->mode == 0 && status == HAL_OK)
	{
		status = read_motor_data(id, 0x0A, hcan);
		osDelay(5);
	}
	status = dm_motor_magic(id, DM_ENABLE_FLAG, hcan);
	// if mode is not 1, The motor will not enabled, so we do not check mode
	return status;
}

HAL_StatusTypeDef DMMotor_DeInit(DM_Motor_CAN_t which_can, uint16_t id)
{
	CAN_HandleTypeDef *hcan;
	if (which_can == USE_CAN1)
	{
		hcan = &hcan1;
	}
	else if (which_can == USE_CAN2)
	{
		hcan = &hcan2;
	}
	else
	{
		while(1);
		// What Fucking Thing are you doing
		return HAL_ERROR;
	}
	if (id == 0 || id > 16)
	{
		while(1);
		// What Fucking Thing are you doing
		return HAL_ERROR;
	}
	return dm_motor_magic(id, DM_CLEARERR_FLAG, hcan);
}

void DMMotor_ESCDataUnpack(const uint8_t* data, DM_Motor_t* motors){
	uint16_t id = data[0] & 0x0F;
	DM_Motor_t * motor = motors + id;
	motor->receive_time = HAL_GetTick();
	if (data[0] == id && data[1] == 0 && (data[2] == 0x33 || data[2] == 0x55))
	{
		// Register Read Recieve
		switch (data[3])
		{
			case 0x0A:
				motor->mode = *((uint32_t*)(data+4));
				break;
			case 0x15:
				motor->P_max = Hex_To_Float((uint32_t*)(data+4)) * RAD_TO_DEGREE;
				break;
			case 0x16:
				motor->V_max = Hex_To_Float((uint32_t*)(data+4));
				break;
			case 0x17:
				motor->T_max = Hex_To_Float((uint32_t*)(data+4));
				break;
			default:
				// fuck you
				break;
		}
	}
	else
	{
		// Motor Feedback Recieve
		motor->state = (DM_Motor_status_t)(data[0] >> 4);
		uint16_t encode_position = (data[1] << 8) | data[2];
		uint16_t encode_velocity = (data[3] << 4) | (data[4] >> 4);
		uint16_t encode_torque = ((data[4] & 0x0F) << 8) | data[5];
		motor->Tmos = (float)data[6];
		motor->Tcoil = (float)data[7];
		
		// Decode Fix-point Numbers
		motor->pos = uint_to_float(encode_position, -motor->P_max, +motor->P_max, 16);
		motor->vel = uint_to_float(encode_velocity, -motor->V_max, +motor->V_max, 12);
		motor->tor = uint_to_float(encode_torque, -motor->T_max, +motor->T_max, 12);
		
		// Update Total Pos System
		if(motor->pos > +motor->P_max / 2 && motor->lastpos < -motor->P_max / 2) motor->refPosCycle --;
		if(motor->pos < -motor->P_max / 2 && motor->lastpos > +motor->P_max / 2) motor->refPosCycle ++;
		motor->lastpos = motor->pos;
		motor->totalpos = motor->pos + motor->refPosCycle * motor->P_max * 2;
	}
}

HAL_StatusTypeDef DMMotor_MIT_SendCommand(DM_Motor_CAN_t which_can, uint16_t id, float position, float velocity, float torque, float Kp, float Kd){
	static uint32_t mailbox;
	static uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	CAN_HandleTypeDef *hcan;
	DM_Motor_t *motor;
	if (which_can == USE_CAN1)
	{
		hcan = &hcan1;
		motor = &DM_Motors_CAN1[id];
	}
	else if (which_can == USE_CAN2)
	{
		hcan = &hcan2;
		motor = &DM_Motors_CAN2[id];
	}
	else
	{
		while(1);
		// What Fucking Thing are you doing
		return HAL_ERROR;
	}
	if (id == 0 || id > 16)
	{
		while(1);
		// What Fucking Thing are you doing
		return HAL_ERROR;
	}
	
	if (motor->state != Enabled)
	{
		return HAL_ERROR;
	}
	
	if (motor->refPosCycle != 0 && Kp > 0)
	{
		motor->state = ParamErr;
		dm_motor_magic(id, DM_DISABLE_FLAG, hcan);
		return HAL_ERROR;
	}
	
	if (position > motor->P_max || position < -motor->P_max || velocity > motor->V_max || velocity < -motor->V_max || torque > motor->T_max || torque < -motor->T_max)
	{
		motor->state = ParamErr;
		dm_motor_magic(id, DM_DISABLE_FLAG, hcan);
		return HAL_ERROR;
	}
	
	if (Kp > KP_MAX || Kp < KP_MIN || Kd > KD_MAX || Kd < KD_MIN)
	{
		motor->state = ParamErr;
		dm_motor_magic(id, DM_DISABLE_FLAG, hcan);
		while(1);
		// What Fucking Thing are you doing
		return HAL_ERROR;
	}
	
	pos_tmp = float_to_uint(position, -motor->P_max, +motor->P_max, 16);
	vel_tmp = float_to_uint(velocity, -motor->V_max, +motor->V_max, 12);
	tor_tmp = float_to_uint(torque, -motor->T_max, +motor->T_max, 12);
	kp_tmp  = float_to_uint(Kp,  KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(Kd,  KD_MIN, KD_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	CAN_TxHeaderTypeDef header = {
		.IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 0x08,
		.StdId = id, .ExtId	= 0, .TransmitGlobalTime = DISABLE
	};
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
	if (status != HAL_OK)
		motor->state = CommFail;
	else 
		motor->transmit_time = HAL_GetTick();
	return status;
}

void DMMotor_General_Init()
{
	memset((void*)DM_Motors_CAN1, 0, sizeof(DM_Motors_CAN1));
	memset((void*)DM_Motors_CAN2, 0, sizeof(DM_Motors_CAN2));
}
