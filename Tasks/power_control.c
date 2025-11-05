#include "power_control.h"

energy_control_t energy;
power_control_t power;

measure_k3_t measure;

static inline float clamp(float x, float lo, float hi) {
	return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static inline float linear_ramp(float x, float lo, float hi) {
	if(lo == hi) return (x >= hi) ? 1.0f : 0.0f;
	return clamp((x-lo) / (hi-lo), 0.0f, 1.0f);
}

void power_measure_k3(void) {
	measure.count ++;
	measure.sum += P_Cha;
	if(measure.count) measure.avg_k3 = measure.sum / measure.count;
}

void power_control_init(void) {
	power_info_t *info = &power.info;
	power_cmd_t *cmd = &power.cmd;
	
	info->rls_wheel = (RLS_t){
		.theta1 = THETA1_INIT, .theta2 = THETA2_INIT,
		.P11 = 1e3f, .P12 = 0.0f, .P22 = 1e3f,
		.lambda = FORGETTING_FACTOR
	};
	
	info->rls_steering = (RLS_t){
		.theta1 = THETA1_INIT, .theta2 = THETA2_INIT,
		.P11 = 1e3f, .P12 = 0.0f, .P22 = 1e3f,
		.lambda = FORGETTING_FACTOR
	};
	
	info->sum_abs_w_wheel = 0.0f;
	info->sum_tau2_wheel  = 0.0f;
	info->sum_tau_w_wheel = 0.0f;
	
	info->sum_abs_w_steering = 0.0f;
	info->sum_tau2_steering  = 0.0f;
	info->sum_tau_w_steering = 0.0f;
	
	info->P_est  = 0.0f;
	info->P_meas = 0.0f;
	
	info->k1_w = info->rls_wheel.theta1;
	info->k2_w = info->rls_wheel.theta2;
	
	info->k1_s = info->rls_steering.theta1;
	info->k2_s = info->rls_steering.theta2;
	
	info->k3 = K3_INIT;
	
	info->cap_on   = 0;
	info->judge_on = 0;
	info->P_ref = 40.0f;
	
	cmd->sum_abs_w_wheel = 0.0f;
	cmd->sum_tau2_wheel  = 0.0f;
	cmd->sum_tau_w_wheel = 0.0f;
	
	cmd->sum_abs_w_steering = 0.0f;
	cmd->sum_tau2_steering  = 0.0f;
	cmd->sum_tau_w_steering = 0.0f;
	
	cmd->P_cmd = 0.0f;
	for(uint8_t i = 1; i <= 4; i++) {
		cmd->alpha[Wheel_ID_list[i]] = 1.0f;
		cmd->P_max[Wheel_ID_list[i]] = 0.0f;
		cmd->alpha[Steering_ID_list[i]] = 1.0f;
		cmd->P_max[Steering_ID_list[i]] = 0.0f;
	}
	cmd->filter_alpha = 0.20f;
	
	measure = (measure_k3_t){.count = 0, .sum = 0.0f, .avg_k3 = 0.0f};
	
	energy = (energy_control_t){
		.E_buf = 0.0f, .err = 0.0f,
		.Kp = 5.0f, .Kd = 0.2f,
		.E_set = cap.cfg.max_energy*0.70f,
		.P_set = 0.0f, .filter_P = 0.15f,
	};
}

static inline void rls_step(void) {
	power_info_t *info = &power.info;
	RLS_t *rls = &info->rls_wheel;

	// build features and target
	float phi1 = info->sum_abs_w_wheel;
	float phi2 = info->sum_tau2_wheel;
	float y = info->P_meas - info->sum_tau_w_wheel - info->k3;
	
	// compute gain
	float sigma0 = rls->P11*phi1*phi1 + 2.0f*rls->P12*phi1*phi2 + rls->P22*phi2*phi2;
	float den = rls->lambda + sigma0; // lambda + Phi^T P Phi
	if(den < 1e-6f) den = 1e-6f;
	float K1 = (rls->P11*phi1 + rls->P12*phi2) / den;
	float K2 = (rls->P12*phi1 + rls->P22*phi2) / den;
	
	// compute error
	float y_est = rls->theta1*phi1 + rls->theta2*phi2;
	float e = y - y_est;
	
	// update parameters
	rls->theta1 += K1 * e;
	rls->theta2 += K2 * e;
	
	// update covariance matrix
	float sigma1 = phi1*rls->P11 + phi2*rls->P12;
	float sigma2 = phi1*rls->P12 + phi2*rls->P22;
	float P11 = rls->P11, P12 = rls->P12, P22 = rls->P22;
	rls->P11 = (P11 - K1*sigma1) / rls->lambda;
	rls->P12 = (P12 - K1*sigma2) / rls->lambda;
	rls->P22 = (P22 - K2*sigma2) / rls->lambda;
	
	info->k1_w = rls->theta1;
	info->k2_w = rls->theta2;
}
#error "Need to think about what is the ground for each rls when we use two distinct rls for two different types of motor!!!!"

void power_control_info_update(void) {
	power_info_t *info = &power.info;
	
	info->sum_abs_w_wheel = 0.0f;
	info->sum_tau2_wheel  = 0.0f;
	info->sum_tau_w_wheel = 0.0f;
	
	info->sum_abs_w_steering = 0.0f;
	info->sum_tau2_steering  = 0.0f;
	info->sum_tau_w_steering = 0.0f;
	
	for(uint8_t i = 1; i <= 4; i++) {
		meas_motor_info_t *motor = &info->motor[Wheel_ID_list[i]];
		
		motor->current = MOTORS[Wheel_ID_list[i]].current * CURRENT_TO_AMPERE;	// [A]
		motor->speed   = MOTORS[Wheel_ID_list[i]].speed * RPM_TO_RADPS;				// [rad/s] 	motor shaft, before gearbox
		motor->torque  = motor->current * TORQUE_CONSTANT;			// [N*m] 		motor shaft, before gearbox
		
		info->sum_abs_w_wheel += fabsf(motor->speed);
		info->sum_tau2_wheel  += motor->torque * motor->torque;
		info->sum_tau_w_wheel += motor->torque * motor->speed;
		
		motor = &info->motor[Steering_ID_list[i]];
		
		motor->current = MOTORS[Steering_ID_list[i]].current * CURRENT_TO_AMPERE;	// [A]
		motor->speed   = MOTORS[Steering_ID_list[i]].speed * RPM_TO_RADPS;				// [rad/s] 	motor shaft, before gearbox
		motor->torque  = motor->current * TORQUE_CONSTANT;			// [N*m] 		motor shaft, before gearbox
		
		info->sum_abs_w_steering += fabsf(motor->speed);
		info->sum_tau2_steering  += motor->torque * motor->torque;
		info->sum_tau_w_steering += motor->torque * motor->speed;
	}
	
	const float k1w = info->rls_wheel.theta1;
	const float k2w = info->rls_wheel.theta2;
	const float k1s = info->rls_steering.theta1;
	const float k2s = info->rls_steering.theta2;
	const float k3 = info->k3;
	info->P_est = info->sum_tau_w_wheel + info->sum_abs_w_steering + 
								k1w*info->sum_abs_w_wheel + k2w*info->sum_tau2_wheel + 
								k1s*info->sum_abs_w_steering + k2s*info->sum_tau2_steering +
								k3;
	// estimated power (model predict, [W])
	
	info->cap_on   = (time() - cap.info.last_receive_time <= 500) ? 1 : 0;
	info->judge_on = 0 ? 1 : 0;
	
	if(info->judge_on) {
		info->P_ref = reference_robot_state.chassis_power_limit;
		info->E_buf = reference_power_heat.chassis_power_buffer;
	}
	
	if(info->cap_on) {
		info->P_meas = cap.feedback.P_Cha;
		// measured power (ground truth, [W])
		
		rls_step();
	}
}
#error "Need to confirm that the current to ampere and current to torque coeffcient for GM6020!!!!"

static inline uint8_t is_cap_ok(void) {
	cap_info_t *info = &cap.info;
	if ((info->soc > CAP_SOC_GATE) && (info->V_ema > CAP_V_GATE)) return 1;
	return 0;
}

void energy_control() {
	cap_info_t *cap_info = &cap.info;
	const float dt = CHASSIS_CTRL_DT;
	const float P_ref = power.info.P_ref;
	const float E_buf = power.info.E_buf;
	
	float dE_buf = (E_buf - energy.E_buf) / fmaxf(dt, 1e-3f);
	energy.E_buf = E_buf;
	
	float err  = sqrtf(fmaxf(energy.E_set, 0.0f)) - sqrtf(fmaxf(cap_info->energy, 0.0f));
	float derr = (err - energy.err) / fmaxf(dt, 1e-3f);
	energy.err = err;
	
	float P_base = P_ref - (energy.Kp*err + energy.Kd*derr);
	
	uint8_t cap_ok = is_cap_ok();
	float P_upper = P_ref + (cap_ok ? CAP_P_INST : 0.0f);
	float P_lower = fmaxf(15.0f, 0.80f * P_ref);

	if(power.info.cap_on) {
		if(dE_buf < DBUF_TRIG || E_buf < BUF_THRESHOLD) {
			P_upper = 0.95f * P_ref;
		}
		else if(E_buf < 55.0f) {
			P_upper = 1.10f * P_ref;
		}
	}
	
	float P_set = clamp(P_base, P_lower, P_upper);

	energy.P_set = energy.P_set + energy.filter_P * (P_set - energy.P_set);
}

void power_control(void) {
	
	if(reference_robot_state.chassis_power_limit > 300) {
		// unlimited power
		// power control off
		return ;
	}
	
	power_info_t *info = &power.info;
	power_cmd_t *cmd = &power.cmd;
	
	float P_set;
	if(info->cap_on && info->judge_on) {
		#if WITH_CAP
		energy_control();
		P_set = energy.P_set;
		#else
		P_set = info->P_ref;
		#endif
	}
	else if(info->cap_on && !info->judge_on) {
		energy_control();
		P_set = fminf(energy.P_set, info->P_ref + 10.0f);
	}
	else if(!info->cap_on && info->judge_on) {
		if(info->E_buf > 50.0f) P_set = info->P_ref + 10.0f;
		else P_set = 0.95f*info->P_ref;
	}
	else {
		P_set = 0.95f*info->P_ref;
	}
	
	const float k1w = info->k1_w, k2w = info->k2_w, k1s = info->k1_s, k2s = info->k2_s, k3 = info->k3;
	const float P_max = P_set;
	const float P_cmd = cmd->sum_tau_w_wheel + cmd->sum_tau_w_wheel +
											k1w*cmd->sum_abs_w_wheel + k2w*cmd->sum_tau2_wheel + k3;
	
	float dP = P_cmd - cmd->P_cmd;
	cmd->P_cmd = P_cmd;
	float m = 0.98f - 0.06f * clamp(dP/100.f, 0.0f, 1.0f);
	float P_guard = m * P_max;
	
	if(info->P_meas > 1.02f * P_max) {
		float beta = P_max / info->P_meas;
		for (uint8_t i = 1; i <= 4; i++) {
			power.current_set[Wheel_ID_list[Wheel_ID_list[i]]] *= beta;
		}
	}
	
	if(P_cmd < P_guard) return ;
	
	// power control
	
	float Werr = 0.0f, Wcmd = 0.0f;
	
	for(uint8_t i = 1; i <= 4; i++) {
		cmd_motor_info_t *motor = &cmd->motor[Wheel_ID_list[Wheel_ID_list[i]]];
		
		Werr += fabsf(motor->err);
		Wcmd += fmaxf(0.0f, motor->P_cmd);
	}
	
	float avg_err = Werr * 0.25f;
	float conf = linear_ramp(avg_err, E_LOWER, E_UPPER);
	
	for(uint8_t i = 1; i <= 4; i++) {
		cmd_motor_info_t *motor = &cmd->motor[Wheel_ID_list[Wheel_ID_list[i]]];

		float w_err  = (Werr > EPS) ? (fabsf(motor->err) / Werr)         : 0.25f;
		float w_prop = (Wcmd > EPS) ? (fmaxf(0.0f, motor->P_cmd) / Wcmd) : 0.25f;
		float weight = conf*w_err + (1.0f-conf)*w_prop;
		cmd->P_max[Wheel_ID_list[Wheel_ID_list[i]]] = P_guard * weight;
	}
	
	for(uint8_t i = 1; i <= 4; i++) {
		cmd_motor_info_t *motor = &cmd->motor[Wheel_ID_list[i]];
		
		// solve alpha
		const float A = k2 * motor->torque * motor->torque;
		const float B = 1.05f * motor->torque * motor->speed;
		const float C = k1 * fabsf(motor->speed) + k3*0.25f - cmd->P_max[Wheel_ID_list[i]];
		
		float alpha = 1.0f;
		
		if(A > 1e-9f) {
			float D = B*B - 4.0f*A*C;
			if(D < 0.0f) {
				alpha = clamp(-B / (2.0f*A), 0.0f, 1.0f);
			}
			else {
				float sqrtD = sqrtf(D);
				float r1 = (-B + sqrtD) / (2.0f*A);
				float r2 = (-B - sqrtD) / (2.0f*A);
				float a1 = clamp(r1, 0.0f, 1.0f);
				float a2 = clamp(r2, 0.0f, 1.0f);
				alpha = fmaxf(a1, a2);
			}
		}
		else {
			if(fabsf(B) > 1e-9f) alpha = clamp(-C / B, 0.0f, 1.0f);
			else alpha = 0.0f;
		}
		
		// rise low pass filter
		float a_prev = cmd->alpha[Wheel_ID_list[i]];
		float a_new  = alpha;
		
		if(a_new < a_prev) cmd->alpha[Wheel_ID_list[i]] = a_new;
		else cmd->alpha[Wheel_ID_list[i]] = a_prev + cmd->filter_alpha * (a_new-a_prev);
		cmd->alpha[Wheel_ID_list[i]] = clamp(cmd->alpha[Wheel_ID_list[i]], 0.0f, 1.0f);
	}
	
	for(uint8_t i = 1; i <= 4; i++) {
		power.current_set[Wheel_ID_list[i]] *= cmd->alpha[Wheel_ID_list[i]];
	}
	
}
