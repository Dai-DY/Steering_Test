#define _COM_C
#include "comm.h"

extern thread_status_t thread_status;

extern UART_HandleTypeDef huart6;

/*================================Begin of Judgement Data ===========================================================*/
uint8_t Judgement_Online=0;
ext_game_robot_state_t reference_robot_state; // data of the robot
ext_power_heat_data_t reference_power_heat;
ext_game_status_t reference_game_state;
ext_bullet_remaining_t reference_bullet_remaining;
ext_game_robot_HP_t  reference_robot_HP;
ext_gimbal_target_t aimbot_gimbal_target;
ext_map_mark_t map_mark;
ext_robot_hurt_t robot_hurt;
adv_energy_detection_t organ_aimbot; // data of aimbot
NX_data_t NX_data;
uint64_t NX_receive_time; // time of receiving aimbot message
NX_robot_state_t robot_state;
ext_nav_status_t nav_status;
ext_slam_control_t slam_control;
aimbot_data_t aimbot_data;
uint64_t SLAM_receive_time;

uint8_t force_nx_transmit = 0;

/*================================End of Judgement Data=============================================================*/

/*================================Begin of Judgement Handler Define =================================================*/
void robot_state_handler(uint8_t * data) {
	memcpy(&reference_robot_state, data, sizeof(ext_game_robot_state_t));
	robot_state.robot_id = reference_robot_state.robot_id;
	Judgement_Online = 200;
}

void power_handler(uint8_t * data) {
	memcpy(&reference_power_heat, data, sizeof(ext_power_heat_data_t));
	Judgement_Online = 200;
}

void robot_HP_handler(uint8_t * data){
  memcpy(&reference_robot_HP,data,sizeof(ext_game_robot_HP_t));
	robot_state.robot_HP = reference_robot_HP;
	Judgement_Online = 200;
}

void mark_handler(uint8_t * data) {
	memcpy(&map_mark, data, sizeof(ext_map_mark_t));
	Judgement_Online = 200;
}

void game_state_handler(uint8_t * data) {
	memcpy(&reference_game_state, data, sizeof(ext_game_status_t));
	Judgement_Online = 200;
}

void robot_hurt_handler(uint8_t *data) {
	memcpy(&robot_hurt, data, sizeof(ext_robot_hurt_t));
	Judgement_Online = 200;
}

/*================================End of Judgement Handler Define====================================================*/

/*================================Begin of NX_data Handler Define =================================================*/

void aimbot_handler(uint8_t * data) {
	memcpy(&aimbot_gimbal_target, data, sizeof(ext_gimbal_target_t));
	organ_aimbot.pit = aimbot_gimbal_target.target_pitch;
	organ_aimbot.yaw = aimbot_gimbal_target.target_yaw;
	organ_aimbot.yaw_spd = aimbot_gimbal_target.target_yaw_speed;
	organ_aimbot.dist = aimbot_gimbal_target.dist;
	NX_receive_time = time();
}

void advanced_aimbot(uint8_t *data) {
	memcpy(&organ_aimbot, data, sizeof(adv_energy_detection_t));
	NX_receive_time = time();
}

void slam_control_handler(uint8_t *data) {
	memcpy(&slam_control, data, sizeof(ext_slam_control_t));
	SLAM_receive_time = time();
}

void nav_status_handler(uint8_t *data) {
	memcpy(&nav_status, data, sizeof(ext_nav_status_t));
	SLAM_receive_time = time();
}

/*================================End of NX_data Handler Define =================================================*/
void Comm_init(void)
{
	comm_init();
	comm_rx_init(cuart1.huart);//for NX
	comm_rx_init(cuart3.huart);//for controller
	comm_rx_init(cuart6.huart);//for reference
	protocol_register(CMD_EXT_POWER_HEAT_DATA, power_handler);
	protocol_register(CMD_EXT_ROBOT_STATE, robot_state_handler);
	protocol_register(CMD_EXT_GAME_STATUS, game_state_handler);
	//protocol_register(CMD_EXT_SHOOT_DATA,handler_bullet);
	protocol_register(CMD_EXT_GIMBAL_TARGET,aimbot_handler);
	protocol_register(CMD_MAP_MARK, mark_handler);
	protocol_register(GIMRUNEAdv_CMD_ID , advanced_aimbot);
	protocol_register(CMD_SLAM_CONTROL , slam_control_handler);
	protocol_register(CMD_NAV_STATUS, nav_status_handler);
	protocol_register(CMD_EXT_ROBOT_HURT, robot_hurt_handler);
	UART_Start_Receive_UnfixedLength_IT(&cuart1, 256);
	UART_Start_Receive_UnfixedLength_IT(&cuart6, 256);
	osThreadCreate("comm_task", Comm_task,NULL, osPriorityNormal, 128);
	aimbot_gimbal_target.target_yaw = 0;
	aimbot_gimbal_target.target_pitch = 0;
	reference_power_heat.chassis_power = 40;
	robot_hurt.hurt_type = 10;
}

void Comm_task(void *args) {
    
    thread_status.uartcom = THREAD_OK;
    while(!(thread_status.uartcom  && //waiting till all task threads are initialized
            thread_status.cancom   &&    
            //thread_status.ui       &&
            thread_status.chassis  &&
            //thread_status.gimbal   &&
            //thread_status.shoot    &&
            thread_status.mainctrl )){
        delay(50);
    }
    
	//float dir = 1;
	uint32_t cnt=0;
	while (1) {
		if(Judgement_Online){
		  Judgement_Online--;
		}
		if(cnt>80){
		  cnt=0;
//			if(!control_mode.chassis_following) {
//				robot_state.robot_id += 20;
//			}
			
			//comm_transmit_data(uart_NX.huart,0xa5,CMD_NX_ROBOT_STATE,(uint8_t *)&robot_state,sizeof(NX_robot_state_t));
			
//			if(!control_mode.chassis_following) {
//				robot_state.robot_id -= 20;
//			}
		}
		else{
		  cnt++;
		}
		delay(10);
		
		SendCapData();
		/*
		if(shoot_data.bullet_speed == 0)
			NX_data.shoot_speed = BULLET_SPD * main_control_mode.fric_on;
		else
			NX_data.shoot_speed = shoot_data.bullet_speed;
			NX_data.curr_pitch = attitude.pitch;
		NX_data.curr_yaw = (-1)* attitude.totalYaw; 
		comm_transmit_data(uart_NX.huart, 0xa5, CMD_NX_DATA, (uint8_t *) &NX_data, sizeof(NX_data_t));
		*/
		
//		float attacked_totalYaw = main_control_mode.attacked_totalYaw;
//		aimbot_data.target_yaw = (attacked_totalYaw - attacked_totalYaw / 360.0f);
//		if (aimbot_data.target_yaw > 180) aimbot_data.target_yaw -= 360;
//		if (aimbot_data.target_yaw < -180) aimbot_data.target_yaw += 360;
//		aimbot_data.target_yaw -= main_control_mode.absolute_x_axis_yaw_angle +  0.143 * time()/1000;
//		aimbot_data.target_dist = organ_aimbot.dist;
//		comm_transmit_data(&huart6, 0xa5, CMD_AIMBOT_DATA, (uint8_t *) &aimbot_data, sizeof(aimbot_data_t));
	}
}
