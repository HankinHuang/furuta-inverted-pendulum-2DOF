/*
 * furuta_swingup.h
 *
 *  Created on: 30 Mar 2026
 *      Author: sumyi
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#ifndef INC_FURUTA_SWINGUP_H_
#define INC_FURUTA_SWINGUP_H_

typedef struct{
	float L_arm;
	float I_arm;
	float lcom_1;
	float I_1;
	float m_1;
	float g;

	float f_v_arm;
	float f_c_arm;
	float k_fc_arm;
	float f_v_1;
	float f_c_1;
	float k_fc_1;
} MechanicalParams;

typedef struct{
	float pendulum_angle_rad;
	float pendulum_velocity_rad_s;
	float arm_angle_rad;
	float arm_velocity_rad_s;
} MeasuredStates;

typedef struct{
	float K;
	float arm_kp;
	float arm_kd;
	float target_energy_offset;
	float tanh_smoothing_factor;
	float v_theta_1_LPF_gain;
	float v_theta_arm_LPF_gain;
} SwingUpTuningParams;

typedef struct{
	float raw_control;
	float saturated_control;
	float pwm_duty;
	int8_t direction;
} SwingUpOutput;

typedef struct{
	  float sample_time_s;
	  float pendulum_counts_per_rev;
	  float motor_counts_per_rev;
	  float motor_gear_ratio;
	  float control_limit;
	  float deadband;
	  float kickstart_pwm_duty;
} SwingUpControllerConfig;

typedef struct{
	int32_t pendulum_counts_total;
	int32_t pendulum_down_counts;
	uint16_t pendulum_counter_prev;

	int32_t arm_counts_total;
	uint16_t motor_counter_prev;
}SwingUpInternalStates;

typedef struct{
	float energy_control;
	float torque;
} SwingUpLogicStates;

typedef struct{
	SwingUpControllerConfig config;
	MeasuredStates measured;
	MechanicalParams mech_params;
	SwingUpTuningParams tuning_params;
	SwingUpOutput output;
	SwingUpInternalStates internal;
	SwingUpLogicStates logic;

	uint8_t control_enabled;
	uint8_t swingup_success;
}SwingUpController;

float swingup_clamp(float value, float min_value, float max_value);
float single_swing_up_feedback_linearisation_controller(MechanicalParams* mech_params, MeasuredStates* states, float torque_limit, float energy_control_input);
float single_swing_up_energy_controller(MechanicalParams* mech_params, MeasuredStates* states, SwingUpTuningParams* tuning_params);
float compute_swingup_torque(SwingUpController* controller);
SwingUpOutput Compute_SwingUpOutput(SwingUpController *controller,
                                          uint16_t pendulum_counter_now,
                                          uint16_t motor_counter_now);
float SwingUp_WrapAngleRad(float angle_rad);
void SwingUp_UpdateMeasuredState(SwingUpController *controller,
                                          uint16_t pendulum_counter_now,
                                          uint16_t motor_counter_now);
void SwingUp_Controller_init(SwingUpController* controller, uint16_t pendulum_counter_now,
        uint16_t motor_counter_now);

void Enable_SwingUp(SwingUpController* controller);
void Disable_SwingUp(SwingUpController* controller);
void SwingUp_arm_angle_velocity_limit(SwingUpController* controller, float arm_angle_limit, float arm_velocity_limit);
void SwingUp_success_check(SwingUpController* controller);

#endif /* INC_FURUTA_SWINGUP_H_ */
