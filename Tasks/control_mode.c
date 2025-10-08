#define CONTROL_MODE_C
#include "control_mode.h"
#include "os.h"

control_mode_t control_mode;


/**
* @breif  init control_mode
* @param  none
* @return none
* @note   set all modes in control_mode to the initial state
*/
void control_mode_init(){
	control_mode.mode         =   OFF_MODE;
	control_mode.gim_chassis  =   GIMBAL_CHASSIS_SEPERATE;
	control_mode.shoot_mode   =   OFF;
	control_mode.fric_spd_lvl =   0;
	control_mode.fric_enable  =   0;
	control_mode.magazine_on  =   1;
	control_mode.AUTO_AIM		  =   0;
	control_mode.singleShoot  =   1;
	control_mode.heat_judge_mode =0;
	control_mode.chassis_following = 0; //0 if chassis not following and organ hitting
	#if !OMNI_WHEEL
		control_mode.chassis_following = 1;
	#endif
	control_mode.chassis_sp   =   0;	
	

	control_mode.q_key_pressed = 0;
	control_mode.f_key_pressed = 0;
	control_mode.m_key_pressed = 0;
	control_mode.g_key_pressed = 0;
	control_mode.rightButton_pressed = 0;
	control_mode.r_key_pressed = 0;
	
	control_mode.auto_cruise = 0;
	control_mode.shift_mode = 0;
	control_mode.z_key_pressed = 0;
	control_mode.shift_key_pressed = 0;
	control_mode.gim_shift_status = 0;
	control_mode.tunnel_mode=0;
	control_mode.turn_mode=0;
}

/**
* @breif  init all flags to 0
* @param  flags: contain all flags used in mode switch
* @return none
* @note
*/
void flag_init(flag_t * flags){
	flags->fric_flag            = 0;
	flags->shoot_flag           = 0; 
	flags->magazine_flag        = 0;
	flags->heat_flag						= 0;
	flags->heat_judge_flag			= 0;
}

/**
* @breif  switch control_mode.mode according to controller
* @param	none
* @return none
* @note   
*/
void control_mode_switch(){
	#if CV_DEBUG || CV_DEBUG_CONTROLLER
	switch(control.triSwitch[L]){
		case MID:
			control_mode.AUTO_AIM = 0;
			control_mode.mode = CONTROLLER_MODE1;
			break;
		
		case UP:
			control_mode.AUTO_AIM = 0;
			control_mode.mode = CONTROLLER_MODE2;
			break;
		
		case DOWN:
			if (!control_mode.AUTO_AIM)
				control_mode.AUTO_AIM = 1;
			control_mode.mode = MOUSE_KEY_MODE;
			break;
		
		default:
			control_mode.mode = OFF_MODE;
	}
	
	
	#else
	if (control.mice.rightButton && (time() - NX_receive_time < 100)) {
		control_mode.AUTO_AIM = 1;
	}
	else if (control_mode.AUTO_AIM == 1){
		control_mode.AUTO_AIM = 0;
	}
	switch(control.triSwitch[L]){
		case MID:
			control_mode.mode = CONTROLLER_MODE1;
			break;
		
		case UP:
			control_mode.mode = CONTROLLER_MODE2;
			break;
		
		case DOWN:
			control_mode.mode = MOUSE_KEY_MODE;
			#if CV_DEBUG
				if (!control_mode.AUTO_AIM)
					control_mode.AUTO_AIM = 1;
			#endif
			break;
		
		default:
			control_mode.mode = OFF_MODE;
	}
	#endif
}

/**
* @breif   switch control_mode.gim_chassis 
*          according to control_mode.mode and controller
* @param   flag
* @return  none
* @note    
*/
void gim_chassis_mode_switch(flag_t * flags){
	switch(control_mode.mode){
		case CONTROLLER_MODE2: // UP
			control_mode.gim_chassis = CHASSIS_FOLLOW_GIMBAL;//CHASSIS_FOLLOW_GIMBAL
			break;
				
		case CONTROLLER_MODE1: // MID
			control_mode.gim_chassis = GIMBAL_CHASSIS_SEPERATE;
			break;
		
		case MOUSE_KEY_MODE:	 // DOWN
			#if OMNI_WHEEL
				if (control_mode.chassis_following){
					control_mode.gim_chassis = CHASSIS_FOLLOW_GIMBAL;
				}
				else if (control_mode.auto_cruise){
					control_mode.gim_chassis = AUTO_CRUISE_MODE;
				}
				else{ // F:
					control_mode.gim_chassis = GIMBAL_CHASSIS_SEPERATE;
				}
			#elif SELF_MADE_MEC
				if(KEY_G){
					if(!control_mode.g_key_pressed) {
						control_mode.chassis_sp = 1;
					}
					control_mode.g_key_pressed = 1;
				}
				else {
					control_mode.g_key_pressed = 0;
				}
				if(control_mode.chassis_sp){
					control_mode.gim_chassis = GIMBAL_CHASSIS_SEPERATE;
				}
				else{
					control_mode.gim_chassis = CHASSIS_FOLLOW_GIMBAL;
				}
			#elif ORDINARY_MEC
				control_mode.gim_chassis = CHASSIS_FOLLOW_GIMBAL;
			#endif
		default:
			control_mode.gim_chassis = GIMBAL_CHASSIS_SEPERATE;
	}
}
	
/**
* @breif   switch control_mode.heat_judge_mode according to roller2
* @param   flag
* @return  none
* @note
*/

void heat_judge(flag_t * flags)
{
	switch(control.roller2)
	{
		case 4:
		default:
			flags->heat_judge_flag = 1;
			break;
		case 1:
			if(control_mode.heat_judge_mode!=2&&flags->heat_judge_flag==1)
			{
				control_mode.heat_judge_mode = 2;
				flags->heat_judge_flag = 0;
			}
			break;
		case 2:
		case 3:
			if(control_mode.heat_judge_mode!=1&&flags->heat_judge_flag==1)
			{
				control_mode.heat_judge_mode = 1;
				flags->heat_judge_flag = 0;
			}
			break;
		case 5:
			if(control_mode.heat_judge_mode!=0&&flags->heat_judge_flag==1)
			{
				control_mode.heat_judge_mode = 0;
				flags->heat_judge_flag = 0;
			}
	}
	if(reference_robot_state.shooter_cooling_limit == 0) {
		control_mode.heat_judge_mode = 2;
	}
}

/**
* @breif   heat judgement according to current heat and heat limit transmit by comm
*          heat judgement mode switch accroding to heat_judge 
* @param   flag
* @return  none
* @note
*/

void muzzle_heat_judgement(flag_t * flags)
{
	switch(control_mode.heat_judge_mode)
	{
		case 0:
		default:
			if(reference_power_heat.shooter_id1_17mm_cooling_heat>reference_robot_state.shooter_cooling_limit- 4 * HEAT_PER_SHOOT)
				flags->heat_flag = 0;
			else if(reference_power_heat.shooter_id1_17mm_cooling_heat<=reference_robot_state.shooter_cooling_limit- 4 * HEAT_PER_SHOOT)
				flags->heat_flag = 1;
			break;
			#if RAGE_MODE			//defined in head file
		case 1:
			if(reference_robot_state.remain_HP>reference_robot_state.max_HP*0.8)
			{
				if(reference_power_heat.shooter_id1_17mm_cooling_heat<reference_robot_state.shooter_id1_17mm_cooling_limit*1.68-4*HEAT_PER_SHOOT)
					flags->heat_flag = 1;
				else
					flags->heat_flag = 0;
			}
			else
			{
				if(reference_power_heat.shooter_id1_17mm_cooling_heat>reference_robot_state.shooter_id1_17mm_cooling_limit-4*HEAT_PER_SHOOT)
				flags->heat_flag = 0;
				else if(reference_power_heat.shooter_id1_17mm_cooling_heat<=reference_robot_state.shooter_id1_17mm_cooling_limit-1*HEAT_PER_SHOOT)
				flags->heat_flag = 1;
			}
			break;
		case 2:
			flags->heat_flag = 1;
			#endif
	}
}

/**
* @breif   switch control_mode.shoot_mode according to
*          control_mode.mode , controller and flags
* @param   flags: contain all flags used in mode switch
* @return  none
* @note
*/

void shoot_mode_switch(flag_t * flags){
	if(control_mode.mode == CONTROLLER_MODE2 && control_mode.fric_spd_lvl){
		switch(control.triSwitch[R]){
			case UP:
				if(flags->shoot_flag && flags->heat_flag) 
				control_mode.shoot_mode = CONTINUE;
				else
				{
					control_mode.shoot_mode = OFF;
				}
				break;
			case MID:
				control_mode.shoot_mode = OFF;
				flags->shoot_flag = 1;
				break;
			case DOWN:
				if(flags->shoot_flag && flags->heat_flag){ //flags->heat_flag
					control_mode.shoot_mode = SINGLE;
					flags->shoot_flag = 0;
				}
				else
				{
					control_mode.shoot_mode = OFF;
					flags->shoot_flag = 0;
				}
				break;
		}
	}
	
	else if (control_mode.mode == MOUSE_KEY_MODE){
		#if !CV_DEBUG_CONTROLLER
		if(KEY_Q) {
			if(!control_mode.q_key_pressed) {
				control_mode.singleShoot = !control_mode.singleShoot;
			}
			control_mode.q_key_pressed = 1;
		}
		else {
			control_mode.q_key_pressed = 0;
		}
	
		if(control.mice.leftButton&& flags->heat_flag&&control_mode.fric_spd_lvl) {
			if(flags -> shoot_flag && flags -> heat_flag) {
				if(control_mode.singleShoot) {
					control_mode.shoot_mode = SINGLE;
				}
				else control_mode.shoot_mode = CONTINUE;
			}
			flags -> shoot_flag = 0;
		}
		else  {
			control_mode.shoot_mode = OFF;
			flags -> shoot_flag = 1;
		}
	  #else
			if (control_mode.fric_spd_lvl){
				switch(control.triSwitch[R]){
					case UP:
					case MID:
						control_mode.shoot_mode = OFF;
						flags->shoot_flag = 1;
						break;
					case DOWN:
						if(flags->shoot_flag&&flags->heat_flag){
							control_mode.shoot_mode = SINGLE;
							flags->shoot_flag = 0;
						}
						else
						{
							control_mode.shoot_mode = OFF;
							flags->shoot_flag = 0;
						}
						break;
					}
			}
			else{
				control_mode.shoot_mode = OFF;
				flags->shoot_flag = 0;
			}
		#endif
	}
}

/**
* @breif   switch control_mode.fric_spd_lvl according to
*          control_mode.mode , controller and flags
* @param   flags: contain all flags used in mode switch
* @return  none
* @note
*/
void fric_mode_switch(flag_t * flags){
	if(control_mode.mode == CONTROLLER_MODE1){
		switch(control.triSwitch[R]){
			case UP:
				if(flags -> fric_flag && control_mode.fric_spd_lvl<FRIC_LVL_MAX \
						&&  shoot.bullet_spd_target[control_mode.fric_spd_lvl + 1] < \
								SHOOT_17mm_SPD_LIMIT) { 
					control_mode.fric_spd_lvl++;
					flags->fric_flag = 0;
				}
				break;
			case MID:
				flags->fric_flag = 1;
				break;
			case DOWN:
				if(flags->fric_flag && control_mode.fric_spd_lvl>0){
					control_mode.fric_spd_lvl --;
					flags->fric_flag = 0;
				}
				break;
		}
	}
	
	else if(control_mode.mode == MOUSE_KEY_MODE){
		if(KEY_E && flags->fric_flag){
			control_mode.fric_enable = 1;
			flags->fric_flag = 0;
		}
		else if(!KEY_E)
			flags->fric_flag = 1;
		
		if(control_mode.fric_enable) { 
			switch(SHOOT_17mm_SPD_LIMIT){
				case 25:
					control_mode.fric_spd_lvl=2;     // FEAT: Custom fric spd setting in game could be designed - Fanch 24.3.16
				
					break;
				default:
					control_mode.fric_spd_lvl=1;
			}
		}
	}
	
}

void magazine_mode_switch(flag_t * flags){
	if(control_mode.mode == CONTROLLER_MODE1){
		if(flags->magazine_flag && control.roller1 == ROLLER1_MAX && !control_mode.magazine_on){
			control_mode.magazine_on =1;
			flags->magazine_flag = 0;
		}
		else if(flags->magazine_flag && control.roller1 == ROLLER1_MAX && control_mode.magazine_on){
			control_mode.magazine_on = 0;
			flags->magazine_flag = 0;
		}
		
		if(control.roller1 != ROLLER1_MAX){
			flags -> magazine_flag =1;
		}
	}
	if(control_mode.mode == MOUSE_KEY_MODE){
		if(flags->magazine_flag && KEY_R & control.key){
			control_mode.magazine_on = !control_mode.magazine_on;
			flags->magazine_flag = 0;
		}
		if(!(KEY_R & control.key)) flags->magazine_flag = 1;
	}
}
	
	void tunnel_switch(void){
	if(KEY_R){
			if(!control_mode.r_key_pressed) {
				control_mode.tunnel_mode = !control_mode.tunnel_mode;
			}
			control_mode.r_key_pressed = 1;
		}
	else {
			control_mode.r_key_pressed = 0;
			}
	}
void turn_switch(void){
	if(KEY_F){
			if(!control_mode.f_key_pressed) {
				control_mode.turn_mode = !control_mode.turn_mode;
			}
			control_mode.f_key_pressed = 1;
		}
	else {
			control_mode.f_key_pressed = 0;
			}
	}
/**
* @breif   main task
* @param	 args: avoid some funtions put parameters to this function
* @return  none
* @note    control the logic level of the task by calling help functions.
*/
void control_mode_task(void * args){
	flag_t flags;
	flag_init(&flags);
	control_mode_init();
	while(1){
		//heat_judge(&flags);
		muzzle_heat_judgement(&flags);
		control_mode_switch();
		gim_chassis_mode_switch(&flags);
		shoot_mode_switch(&flags);
		fric_mode_switch(&flags);
		magazine_mode_switch(&flags);
		tunnel_switch();
		turn_switch();
		delay(5);
	}
}


