#include "main_control.h"
#include "board_transmit.h"
#include "comm.h"

gimbal_info_t receiver_from_chassis;
shoot_info_t receiver_from_gimbal;
control_mode_t key_mouse_control_mode;
kbd_spd_t kbd_state;

float v_keymouse[4],v_x,v_y,v_w;
int v_w_direction;
float ratio_ac=1.0f;
float ratio_sl=1.0f;


thread_status_t thread_status;
// internal, information of mode
main_control_mode_t main_control_mode;

// internal, controller of motor
motor_controller_t motor_controller;


// ****************************************** Begin of Init ******************************************//
void main_control_init() {
	main_control_mode.control_mode = OFF_MODE;
	//main_control_mode.shoot_mode = OFF;
	main_control_mode.chassis_control_mode = CHASSIS_OFF;
	main_control_mode.fric_on = 0;
	main_control_mode.direction_mode = FOWARD_MODE;
	
	key_mouse_control_mode.c_key_cd = 0;
	key_mouse_control_mode.Top_enable = 0;
	key_mouse_control_mode.z_key_cd = 0;
	key_mouse_control_mode.Top_enable_fast = 0;
	key_mouse_control_mode.x_key_pressed = 1;
	v_w_direction=-1;
	
	thread_status.mainctrl = THREAD_OK;
    while(!(thread_status.uartcom  && //waiting till all task threads are initialized
            thread_status.cancom   &&    
            //thread_status.ui       &&
            thread_status.chassis  &&
            //thread_status.gimbal   &&
            //thread_status.shoot    &&
            thread_status.mainctrl )){
        delay(50);
    }
	// init totalAngle of motors (the initial motor angle as 0 degree)
}

// ****************************************** End of Init ******************************************//
	
// ****************************************** Begin of Mode Switch ******************************************//
void control_mode_switch() {
	if(!control.online) main_control_mode.control_mode = OFF_MODE;	
	else {
		switch(control.triSwitch[0]){
			case UP:
				main_control_mode.control_mode = CONTROLLER_MODE;
				break;
			
			case MID:			
				main_control_mode.control_mode = MOUSE_KEY_MODE;
				break;
			case DOWN:
				main_control_mode.control_mode = OFF_MODE;
				break;
		}		
	}
}

////the control of chassis start from here



///stop before here

// ****************************************** End of Mode Switch ******************************************//


// ****************************************** Begin of Target Send ******************************************//


void chassis_target_send() {
	// send target about vx and vy (which is relative to gimbal)
	switch(main_control_mode.control_mode) {
		case CONTROLLER_MODE:
			chassis_set_command(control.channel[1] *_CHASSIS_MAX_VX_SPEED,
													control.channel[0] *_CHASSIS_MAX_VY_SPEED,
													0,
													FOLLOW_ON_0_DEGREE);
			break;
		case MOUSE_KEY_MODE:
			chassis_set_command(control.channel[1] *_CHASSIS_MAX_VX_SPEED,
													control.channel[0] *_CHASSIS_MAX_VY_SPEED,
													_CHASSIS_MAX_VR_SPEED,
													FOLLOW_OFF);
			break;
		case OFF_MODE:
		default:
			chassis_off();
			break;
	}
}
// ****************************************** End of Target Send ******************************************//

// ****************************************** Begin of Main Task ******************************************//
void main_control_task(void * args) {
	main_control_init();
	
	while(1) {
		control_mode_switch();
		
		chassis_target_send();

		delay(5);
	}
}

// ****************************************** End of Main Task ******************************************//
