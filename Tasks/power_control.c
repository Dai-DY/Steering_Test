#ifndef POWER_CONTROL_C_
#define POWER_CONTROL_C_

#include "power_control.h"
#include "board_transmit.h"
#define MTC 20/(16384*100)
#define MAX_POWER_LIMIT 0.97f   //this one value range (0,1], recommend [0.85,0.95]
#define MAX_POWER_LIMIT_BUDDERLESS 0.80f
#define ADJUSTMENT_FACTOR 1.0f
#define MAX_CURRENT 15000
#define DECRADIO 1.5

extern ext_power_heat_data_t reference_power_heat;
float EMA_cur_power = .0f;
float EMA_rec_power = .0f;
float EMA_cmd_power = .0f;
float total_cur_power = 0;
float total_cmd_power = 0;

power_control_t power_control;
test_t test;

inline static float get_Cap_energy(float lowest_V){
  return 0.5f*V_Cap*V_Cap*(20.0f/3.0f)-0.5f*lowest_V*lowest_V*(20.0f/3.0f);
}


void power_control_init(){
  power_control.k=0.20;
	power_control.buffer_to_power = 0.8f;//0.8f
	power_control.last_cmd_power=P_Cha;//reference_power_heat.chassis_power;
	power_control.cmd_power=0;
	power_control.ema_power_to_walts=0.0f;
	power_control.max_power_limit=MAX_POWER_LIMIT;            
	power_control.last_cap_voltage=V_Cap;
	
	#if SELF_MADE_MEC
		power_control.voltage_limit=12.0f;
	#else
		power_control.voltage_limit=14.0f;
	#endif
}

void update_power_K(){
	power_control.cur_cap_voltage=V_Cap;
	total_cur_power = 0.0f;
	for(int i=1;i<=4;i++){
		if(abs(power_control.cur_speed[i])<100){
		  power_control.cur_speed[i]=power_control.cur_speed[i]>=0?100:-100;
		}
		if(abs(power_control.cur_speed[i])>2000)
		  power_control.cur_speed[i]=power_control.cur_speed[i]>0?2000:-2000;
		total_cur_power+=(float)abs(CAN1_Motors[i].current*power_control.cur_speed[i])*MTC;
	}
	if(total_cur_power==0){
	  total_cur_power = 0.10f;
	}
	else {
		EMA_rec_power = EMA_rec_power * (1 - power_control.k) + power_control.cur_chassis_power * power_control.k;
		EMA_cur_power = EMA_cur_power * (1 - power_control.k) + total_cur_power * power_control.k;
		power_control.ema_power_to_walts = EMA_rec_power / EMA_cur_power;
		VAL_LIMIT(power_control.ema_power_to_walts,0,1);
	}
	power_control.cur_chassis_power =P_Cha;//;reference_power_heat.chassis_power;
}

void calc_cmd_power(){
	  total_cmd_power = 0.0f;
	  float adjustment_factor = ADJUSTMENT_FACTOR;
	  for(int i=1;i<=4;i++){
		if(abs(power_control.cur_speed[i])<3){
		  power_control.cur_speed[i]=power_control.cur_speed[i]>=0?3:-3;
		}
		if(abs(power_control.cur_speed[i])>2000)
		  power_control.cur_speed[i]=power_control.cur_speed[i]>0?2000:-2000;
		  total_cmd_power+=(float)abs(CAN1_MotorCurrents[i]*power_control.cur_speed[i])*MTC;
	  }
	  if(total_cmd_power==0){
	    total_cmd_power = 2.0f;
	  }
	  total_cmd_power = total_cmd_power * adjustment_factor;
    power_control.cmd_power =  total_cmd_power* power_control.ema_power_to_walts;
}

void power_control_main(){
		#if POWER_CONTROL
	  for(int i=1;i<5;i++){
			power_control.cur_speed[i]=CAN1_Motors[i].speed;
		}
		#if POWER_TEST
		test.wheelspd1=CAN1_Motors[1].speed/10;
		test.wheelspd2=CAN1_Motors[2].speed/10;
		test.wheelspd3=CAN1_Motors[3].speed/10;
		test.wheelspd4=CAN1_Motors[4].speed/10;
		#endif
		update_power_K();
    for (int i = 1; i <= 4; i++) {
			CAN1_MotorCurrents[i] = PID_calc(&chassis.wheel_pid[i], chassis.info.speed_get[i], chassis.command.speed_set[i]);
		}
		#if POWER_TEST
		test.wheelcur1=CAN1_MotorCurrents[1]/100;
		test.wheelcur2=CAN1_MotorCurrents[2]/100;
		test.wheelcur3=CAN1_MotorCurrents[3]/100;
		test.wheelcur4=CAN1_MotorCurrents[4]/100;
		#endif
		calc_cmd_power();
		
		float buffer;
		power_control.max_power = reference_robot_state.chassis_power_limit;
		if(Judgement_Online) {
			VAL_LIMIT(power_control.max_power, 15, 130);
		}
		else {
			power_control.max_power = 45;
		}
		if(V_Cap>=power_control.voltage_limit){
			if( main_control_mode.control_mode == MOUSE_KEY_MODE){
					if(KEY_SHIFT){
						if(reference_power_heat.chassis_power_buffer>10u){
							power_control.max_power+=7*V_Cap;
							power_control.max_power*=power_control.max_power_limit;
						}
						else{
							power_control.max_power+=7*V_Cap;
							power_control.max_power*=MAX_POWER_LIMIT_BUDDERLESS;
						}
					}
					else{
						if(reference_power_heat.chassis_power_buffer>10u){
							power_control.max_power*=power_control.max_power_limit;
						}
						else{
							power_control.max_power*=MAX_POWER_LIMIT_BUDDERLESS;
						}
					}
			}
			else{  //controller
				if(reference_power_heat.chassis_power_buffer>10u){
						power_control.max_power+=7*V_Cap;
						power_control.max_power*=power_control.max_power_limit;
				}
				else{
					  power_control.max_power+=7*V_Cap;
						power_control.max_power*=MAX_POWER_LIMIT_BUDDERLESS;
				}
			} 
		}
		else{ //<limit v_cap
				power_control.max_power*=MAX_POWER_LIMIT_BUDDERLESS;
		}
    if(power_control.cmd_power>power_control.max_power){
			float reduce_k = power_control.max_power/power_control.cmd_power;
			power_control.cmd_power_output=power_control.cmd_power*reduce_k;
		  for (int i = 1; i <= 4; i++) {
			  CAN1_MotorCurrents[i] *= reduce_k; 
				//chassis.spd_set[i+4]=(chassis.spd_set[i+4]+(1-DECRADIO)*chassis.spd_set[i])*reduce_k/chassis.spd_set[i+4];
				//chassis.spd_set[i]=chassis.spd_set[i]*reduce_k*DECRADIO;
				//CAN1_MotorCurrents[i] = PID_calc(&chassis.mtr[i].pid_spd, chassis.spd_get[i], chassis.spd_set[i]+chassis.spd_set[i+4]);
		  }
 		}
		else{
			power_control.cmd_power_output=power_control.cmd_power;
		}
		for (int i = 1; i <= 4; i++) {
			  VAL_LIMIT(CAN1_MotorCurrents[i],-MAX_CURRENT, MAX_CURRENT);
		}
		#if POWER_TEST
		test.wheelcurk1=CAN1_MotorCurrents[1]/100;
		test.wheelcurk2=CAN1_MotorCurrents[2]/100;
		test.wheelcurk3=CAN1_MotorCurrents[3]/100;
		test.wheelcurk4=CAN1_MotorCurrents[4]/100;
		#endif
		#if POWER_TEST
		test.aver ++;                                     //calculate average power
		test.total_pow +=reference_power_heat.chassis_power;
		if(test.aver%50==0){
			test.aver_pow = test.total_pow/50;
			test.aver = 0;
			test.total_pow = 0;
		}
		#endif 
		#else
		for (int i = 1; i <= 4; i++) {
			CAN1_MotorCurrents[i] = PID_calc(&chassis.mtr[i].pid_spd, chassis.spd_get[i], chassis.spd_set[i]);
		}	
		#if POWER_TEST
		test.wheelcur1=CAN1_MotorCurrents[1]/100;
		test.wheelcur2=CAN1_MotorCurrents[2]/100;
		test.wheelcur3=CAN1_MotorCurrents[3]/100;
		test.wheelcur4=CAN1_MotorCurrents[4]/100;
		#endif
		#if POWER_TEST
		test.wheelspd1=CAN1_Motors[1].speed/10;
		test.wheelspd2=CAN1_Motors[2].speed/10;
		test.wheelspd3=CAN1_Motors[3].speed/10;
		test.wheelspd4=CAN1_Motors[4].speed/10;
		#endif
		#endif
}
#endif //POWER_CONTROL_C_

