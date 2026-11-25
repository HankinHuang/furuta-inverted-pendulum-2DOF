/*
 * furuta_swingup.c
 *
 *  Created on: 30 Mar 2026
 *      Author: sumyi
 */

#include "furuta_swingup.h"

float single_swing_up_feedback_linearisation_controller(MechanicalParams* mech_params, MeasuredStates* states, float torque_limit, float energy_control_input)
{

    float L_arm = mech_params->L_arm;
	float I_arm = mech_params->I_arm;
	float lcom_1 = mech_params->lcom_1;
	float I_1 = mech_params->I_1;
	float m_1 = mech_params->m_1;
	float g = mech_params->g;

	float f_v_1 = mech_params->f_v_1;
	float f_c_1 = mech_params->f_c_1;
	float k_fc_1 = mech_params->k_fc_1;

	float v_theta_arm = states->arm_velocity_rad_s;
	float theta_1 = states->pendulum_angle_rad;
	float v_theta_1 = states->pendulum_velocity_rad_s;

	float L_arm_sq = L_arm * L_arm;
	float lcom_1_sq = lcom_1 * lcom_1;
	float lcom_1_pow3 = lcom_1 * lcom_1 * lcom_1;
	float m_1_sq = m_1 * m_1;
	float v_theta_arm_sq = v_theta_arm * v_theta_arm;
	float v_theta_1_sq = v_theta_1 * v_theta_1;
	float sin_theta_1_sq = sinf(theta_1) * sinf(theta_1);
	float cos_theta_1_sq = cosf(theta_1) * cosf(theta_1);

    float A = m_1 * L_arm_sq + I_arm + m_1 * lcom_1_sq * sin_theta_1_sq
            - ((m_1_sq * L_arm_sq * lcom_1_sq * cos_theta_1_sq) / (m_1 * lcom_1_sq + I_1));

    float B_1 = ((0.5f) * m_1_sq * L_arm * lcom_1_pow3 * sinf(2.0f * theta_1) * cosf(theta_1) * v_theta_arm_sq)
              / (m_1 * lcom_1_sq + I_1);
    float B_2 = -m_1 * L_arm * lcom_1 * sinf(theta_1) * v_theta_1_sq;
    float B_3 = m_1 * lcom_1_sq * sinf(2.0f * theta_1) * v_theta_1 * v_theta_arm;
    float B_4 = ((-m_1 * L_arm * lcom_1 * cosf(theta_1) * f_v_1) / (m_1 * lcom_1_sq + I_1)) * v_theta_1;
    float B_5 = ((-m_1 * L_arm * lcom_1 * cosf(theta_1) * f_c_1 * tanhf(k_fc_1 * v_theta_1)) / (m_1 * lcom_1_sq + I_1)) * v_theta_1;
    float B_6 = ((m_1_sq * g * L_arm * lcom_1_sq * sinf(theta_1) * cosf(theta_1)) / (m_1 * lcom_1_sq + I_1));
    float B = B_1 + B_2 + B_3 + B_4 + B_5 + B_6;

    float torque_swing_up_uncapped = A * (energy_control_input + B / A);
    float torque_total_capped = fmaxf(-torque_limit, fminf(torque_limit, torque_swing_up_uncapped));

    return torque_total_capped;
}

float single_swing_up_energy_controller(MechanicalParams* mech_params,
                                        MeasuredStates* states,
                                        SwingUpTuningParams* tuning_params)
{
    float L_arm = mech_params->L_arm;
    float lcom_1 = mech_params->lcom_1;
    float I_1 = mech_params->I_1;
    float m_1 = mech_params->m_1;
    float g = mech_params->g;

    /* The measured arm angle is stored as a continuous angle for state estimation.
     * For the arm PD term in the swing-up controller, we only need the local arm
     * deviation, so we wrap it back to [-pi, pi). This prevents the proportional
     * term from growing artificially after multiple arm revolutions. */
    float theta_arm = SwingUp_WrapAngleRad(states->arm_angle_rad);
    float v_theta_arm = states->arm_velocity_rad_s;
    float theta_1 = states->pendulum_angle_rad;
    float v_theta_1 = states->pendulum_velocity_rad_s;

    float K = tuning_params->K;
    float k_p = tuning_params->arm_kp;
    float k_d = tuning_params->arm_kd;
    float target_energy_offset = tuning_params->target_energy_offset;
    float tanh_smoothing_factor = tuning_params->tanh_smoothing_factor;

    float lcom_1_sq = lcom_1 * lcom_1;
    float v_theta_1_sq = v_theta_1 * v_theta_1;

    float E = 0.5f * (m_1 * lcom_1_sq + I_1) * v_theta_1_sq
            + m_1 * g * lcom_1 * cosf(theta_1);

    float E_d = m_1 * g * lcom_1;
    float E_d_margin = target_energy_offset * E_d;
    E_d = E_d + E_d_margin;

    float tanh_argument_state_dep = (E - E_d) * cosf(theta_1) * v_theta_1;
    float tanh_argument_overall = tanh_smoothing_factor * tanh_argument_state_dep;

    float pendulum_energy_control =
        (1.0f / (m_1 * L_arm * lcom_1)) * K * tanhf(tanh_argument_overall);

    float arm_PD_control = (k_p * theta_arm + k_d * v_theta_arm);
    float energy_control_output = pendulum_energy_control + arm_PD_control;

    return energy_control_output;
}

float compute_swingup_torque(SwingUpController* controller)
{
	if (controller == NULL) { return 0.0f; }

	float energy_control_output = single_swing_up_energy_controller(&controller->mech_params, &controller->measured, &controller->tuning_params);
	float torque = single_swing_up_feedback_linearisation_controller(&controller->mech_params, &controller->measured, controller->config.control_limit, energy_control_output);
	controller->logic.energy_control = energy_control_output;
	controller->logic.torque = torque;
	return torque;
}

SwingUpOutput Compute_SwingUpOutput(SwingUpController *controller,
                                    uint16_t pendulum_counter_now,
                                    uint16_t motor_counter_now)
{
	SwingUpOutput output;
  float abs_control;

  output.raw_control = 0.0f;
  output.saturated_control = 0.0f;
  output.pwm_duty = 0.0f;
  output.direction = 0;

  if (controller == NULL) { return output; }

  SwingUp_UpdateMeasuredState(controller, pendulum_counter_now, motor_counter_now);

  if (controller->control_enabled == 0U)
  {
    return output;
  }

  output.raw_control = compute_swingup_torque(controller);

  output.saturated_control = swingup_clamp(output.raw_control,
                                           -controller->config.control_limit,
                                           controller->config.control_limit);

  abs_control = fabsf(output.saturated_control);

  if (abs_control < controller->config.deadband)
  {
    output.direction = 0;
    output.pwm_duty = 0.0f;
  }
  else
  {
    output.direction = (output.saturated_control > 0.0f) ? -1 : 1;
    output.pwm_duty = swingup_clamp(abs_control / controller->config.control_limit, 0.0f, 1.0f);
  }

  return output;
}

float swingup_clamp(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

float SwingUp_WrapAngleRad(float angle_rad)
{
  const float pi = 3.14159265358979323846f;
  const float two_pi = 6.28318530717958647692f;
  float wrapped = fmodf(angle_rad + pi, two_pi);

  if (wrapped < 0.0f)
  {
    wrapped += two_pi;
  }

  return wrapped - pi;
}

void SwingUp_UpdateMeasuredState(SwingUpController *controller,
                                 uint16_t pendulum_counter_now,
                                 uint16_t motor_counter_now)
{
  int16_t pendulum_delta_counts;
  int16_t motor_delta_counts;
  float dt_s;
  float pendulum_angle_continuous_rad;
  const float two_pi = 6.28318530717958647692f;

  if (controller == NULL) { return; }

  dt_s = controller->config.sample_time_s;

  pendulum_delta_counts = (int16_t)(pendulum_counter_now - controller->internal.pendulum_counter_prev);
  controller->internal.pendulum_counter_prev = pendulum_counter_now;
  controller->internal.pendulum_counts_total += pendulum_delta_counts;

  pendulum_angle_continuous_rad =
      ((float)(controller->internal.pendulum_counts_total - controller->internal.pendulum_down_counts)
       / controller->config.pendulum_counts_per_rev) * two_pi;

  controller->measured.pendulum_angle_rad =
		  - SwingUp_WrapAngleRad(pendulum_angle_continuous_rad - 3.14159265358979323846f);

  /* Keep the arm position as a continuous accumulated angle instead of wrapping it
   * to [-pi, pi). This avoids artificial jumps at the wrap boundary and gives a
   * cleaner position/velocity estimate from the encoder counts. A wrapped version
   * can still be generated later wherever the controller specifically needs a
   * bounded arm angle. */
  motor_delta_counts = (int16_t)(motor_counter_now - controller->internal.motor_counter_prev);
  controller->internal.motor_counter_prev = motor_counter_now;
  controller->internal.arm_counts_total += motor_delta_counts;

  controller->measured.arm_angle_rad =
      -((float)controller->internal.arm_counts_total /
       (controller->config.motor_counts_per_rev * controller->config.motor_gear_ratio)) * two_pi;

  if (dt_s > 0.0f)
  {
	  float previous_pendulum_velocity_rad_s = controller->measured.pendulum_velocity_rad_s;
	  float previous_arm_velocity_rad_s = controller->measured.arm_velocity_rad_s;

	  float current_pendulum_velocity_rad_s = -((float)pendulum_delta_counts / controller->config.pendulum_counts_per_rev) * two_pi / dt_s;
    controller->measured.pendulum_velocity_rad_s = (1.0f - controller->tuning_params.v_theta_1_LPF_gain) * current_pendulum_velocity_rad_s
                                                 + controller->tuning_params.v_theta_1_LPF_gain * previous_pendulum_velocity_rad_s;

    float current_arm_velocity_rad_s = -((float)motor_delta_counts /
            (controller->config.motor_counts_per_rev * controller->config.motor_gear_ratio)) * two_pi / dt_s;
    controller->measured.arm_velocity_rad_s = (1.0f - controller->tuning_params.v_theta_arm_LPF_gain) * current_arm_velocity_rad_s
                                            + controller->tuning_params.v_theta_arm_LPF_gain * previous_arm_velocity_rad_s;
  }
  else
  {
    controller->measured.pendulum_velocity_rad_s = 0.0f;
    controller->measured.arm_velocity_rad_s = 0.0f;
  }
}

void SwingUp_Controller_init(SwingUpController* controller, uint16_t pendulum_counter_now,
                             uint16_t motor_counter_now)
{
	if (controller == NULL) { return; }

	controller->measured.pendulum_angle_rad = 0.0f;
	controller->measured.pendulum_velocity_rad_s = 0.0f;
	controller->measured.arm_angle_rad = 0.0f;
	controller->measured.arm_velocity_rad_s = 0.0f;

	controller->internal.pendulum_counts_total = 0;
	controller->internal.pendulum_down_counts = 0;
	controller->internal.pendulum_counter_prev = pendulum_counter_now;
	controller->internal.arm_counts_total = 0;
	controller->internal.motor_counter_prev = motor_counter_now;

	controller->control_enabled = 0U;
	controller->swingup_success = 0U;
}

void Enable_SwingUp(SwingUpController* controller)
{
	if (controller == NULL) { return; }

	controller->output.raw_control = 0.0f;
	controller->output.saturated_control = 0.0f;
	controller->output.pwm_duty = 0.0f;
	controller->output.direction = 0U;
	controller->control_enabled = 1U;
	controller->swingup_success = 0U;
}

void Disable_SwingUp(SwingUpController* controller)
{
	if (controller == NULL) { return; }

	controller->output.raw_control = 0.0f;
	controller->output.saturated_control = 0.0f;
	controller->output.pwm_duty = 0.0f;
	controller->output.direction = 0U;
	controller->control_enabled = 0U;
}

void SwingUp_arm_angle_velocity_limit(SwingUpController* controller, float arm_angle_limit, float arm_velocity_limit)
{
    if (controller == NULL) { return; }

    if ((fabsf(SwingUp_WrapAngleRad(controller->measured.arm_angle_rad)) > arm_angle_limit) ||
        (fabsf(controller->measured.arm_velocity_rad_s) > arm_velocity_limit))
    {
        Disable_SwingUp(controller);
    }
}

void SwingUp_success_check(SwingUpController* controller)
{
	if (controller == NULL) { return; }

	if (fabsf(controller->measured.pendulum_angle_rad) < 0.2f &&
        fabsf(controller->measured.pendulum_velocity_rad_s) < 1.0f)
	{
		controller->swingup_success = 1U;
	}
}
