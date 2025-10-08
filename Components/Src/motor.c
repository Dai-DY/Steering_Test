#include "stm32f4xx_hal.h"
#include "motor.h"
#include "comm.h"

extern CAN_HandleTypeDef hcan1, hcan2;

void CanMotor_Init(void)
{
    CAN_FilterTypeDef filter;

    /* allow standard data frame whose id is between 0x200 and 0x20f */
    filter.FilterIdHigh = ESC_BASE_ID << 5;
    filter.FilterIdLow = CAN_ID_STD << 2 | CAN_RTR_DATA << 1;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FilterFIFO0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14;
    /* config the filter on CAN1 */
    filter.FilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    /* config the filter on CAN2 */
    filter.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &filter);
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
		HAL_CAN_Start(&hcan1);
		HAL_CAN_Start(&hcan2);
}

void CanMotor_ESCDataUnpack(uint8_t const *data, CanMotor *motor)
{
    motor->lastAngle = motor->angle;
    motor->angle = (uint16_t)data[0] << 8 | data[1];
    /* deltaAngle = angle - lastAngle (mod 8192) */
    uint16_t deltaAngle = (uint16_t)(motor->angle - motor->lastAngle) & 0x1fff;
    motor->speed = (int16_t)data[2] << 8 | data[3];
    motor->current = (int16_t)data[4] << 8 | data[5];
    motor->temperature = data[6];
    motor->totalAngle += deltaAngle;
    /* if deltaAngle >= 4096 */
    if (deltaAngle & 0x1000)
    {
        /* in which case the motor went backward */
        motor->totalAngle -= MOTOR_ENCODER_RESOLUTION;
    }
}

CanMotor CAN1_Motors[13];
CanMotor CAN2_Motors[13];
int16_t CAN1_MotorCurrents[13];
int16_t CAN2_MotorCurrents[13];

static inline void packMotorCurrents(const int16_t currents[4], uint8_t buffer[8])
{
    buffer[0] = currents[0] >> 8;
    buffer[1] = currents[0];
    buffer[2] = currents[1] >> 8;
    buffer[3] = currents[1];
    buffer[4] = currents[2] >> 8;
    buffer[5] = currents[2];
    buffer[6] = currents[3] >> 8;
    buffer[7] = currents[3];
}

void CanMotor_SendCurrent(CAN_Motor_Selection selection)
{
    static CAN_TxHeaderTypeDef header[] = {
        {.StdId = CURRENTS_1_TO_4, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8},
        {.StdId = CURRENTS_5_TO_8, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8},
        {.StdId = CURRENTS_9_TO_12, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8},
        {.StdId = CURRENTS_1_TO_4, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8},
        {.StdId = CURRENTS_5_TO_8, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8},
        {.StdId = CURRENTS_9_TO_12, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8}};
    static CAN_HandleTypeDef *hcan[] = {
        &hcan1, &hcan1, &hcan1, &hcan2, &hcan2, &hcan2};
    static int16_t *currents[] = {
        CAN1_MotorCurrents + 1, CAN1_MotorCurrents + 5, CAN1_MotorCurrents + 9,
        CAN2_MotorCurrents + 1, CAN2_MotorCurrents + 5, CAN2_MotorCurrents + 9};
    static uint32_t mailbox;
    static uint8_t buf[6][8];

    packMotorCurrents(currents[selection], buf[selection]);

    /* if more than three packets come within 130us, adding new tx msg will fail */
    HAL_CAN_AddTxMessage(hcan[selection], &header[selection], buf[selection], &mailbox);
}

float V_Cap;
float P_Cha;

void CAP_Unpack(const uint8_t * packet){
	uint16_t * buf = (uint16_t *) packet;
	int16_t * buf_ = (int16_t *) packet;
	P_Cha = buf_[0] > 1 ? (float) buf_[0]/10.0f : 1.0f;
	V_Cap = (float)buf[1]/100.0f;
}


void SendCapData(){
#if WULIE_CAPACITY
	static uint8_t buf[8];
	uint16_t power;
	if(reference_power_heat.chassis_power_buffer>=20u )	{
	  power = (uint16_t )(reference_robot_state.chassis_power_limit+reference_power_heat.chassis_power_buffer * 0.2f)*100;//reference_robot_state.chassis_power_limit
	}
	else{
	  power = (uint16_t )(reference_robot_state.chassis_power_limit+(reference_power_heat.chassis_power_buffer-15)*0.2f)*100;//
	}
	if(reference_robot_state.chassis_power_limit == 255) {
			power = 8000.0f;
		}
	
//	if(reference_robot_state.chassis_power_limit > 160) {
//		power = 16000.0f;
//	}
//	power = 10000.0f;//power over control
	buf[0]=power>>8u;
	buf[1]=power;
	static uint32_t mailbox;
	static CAN_TxHeaderTypeDef header={.StdId=0x210,.IDE = CAN_ID_STD, .RTR=CAN_RTR_DATA,.DLC=8};
	HAL_CAN_AddTxMessage(&hcan1,&header,(uint8_t *)&buf,&mailbox);
#else 
	static struct {
		uint8_t maxPower;
		uint8_t _[7];
	} buf;	
	buf.maxPower = reference_robot_state.chassis_power_limit;
	static uint32_t mailbox;
	static CAN_TxHeaderTypeDef header={.StdId=0x210,.IDE = CAN_ID_STD, .RTR=CAN_RTR_DATA,.DLC=8};
	HAL_CAN_AddTxMessage(&hcan1,&header,(uint8_t *)&buf,&mailbox);
#endif
}
