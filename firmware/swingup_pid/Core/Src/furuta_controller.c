#include "furuta_controller.h"

#include <math.h>
#include <stddef.h>

/* 简单限幅函数 */
static float furuta_clamp(float value, float min_value, float max_value)
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


static float furuta_pid_step(FurutaPidState *pid, float error, float dt_s)
{
  float proportional = 0.0f;
  float integral = 0.0f;
  float derivative = 0.0f;
  float derivative_alpha;
  float derivative_input;
  float output;

  if (pid == NULL || dt_s <= 0.0f)
  {
    return 0.0f;
  }

  proportional = pid->kp * error;
  pid->integral += error * dt_s;
  integral = pid->ki * pid->integral;
  derivative_alpha = 1.0f - (pid->derivative_filter_n * dt_s);
  derivative_alpha = furuta_clamp(derivative_alpha, 0.0f, 1.0f);
  derivative_input = pid->kd * pid->derivative_filter_n * (error - pid->prev_error);
  pid->derivative_state = (derivative_alpha * pid->derivative_state) + derivative_input;
  derivative = pid->derivative_state;

  pid->prev_error = error;

  output = proportional + integral + derivative;

  return furuta_clamp(output, pid->output_min, pid->output_max);
}

static float furuta_compute_raw_control(FurutaController *controller)
{
  float arm_error = 0.0f;
  float pendulum_error = 0.0f;
  float motor_command = 0.0f;
  uint8_t divider;
  float outer_dt_s;

  if (controller == NULL)
  {
    return 0.0f;
  }

  divider = controller->config.arm_outer_loop_divider;
  if (divider == 0U)
  {
    divider = 1U;
  }

  controller->arm_outer_loop_counter++;

  if (controller->arm_outer_loop_counter >= divider)
  {
    controller->arm_outer_loop_counter = 0U;

    arm_error = controller->arm_hold_angle_rad - controller->measured.arm_angle_rad;
    outer_dt_s = controller->config.sample_time_s * (float)divider;

    controller->arm_reference_rad =
        -furuta_pid_step(&controller->config.arm_pid,
                        arm_error,
                        outer_dt_s);
  }

  const float pendulum_trim_rad = -0.0f;
  pendulum_error = (controller->arm_reference_rad + pendulum_trim_rad)
                   - controller->measured.pendulum_angle_rad;

  motor_command = furuta_pid_step(&controller->config.pendulum_pid,
                                  pendulum_error,
                                  controller->config.sample_time_s);

  return -motor_command;
}

FurutaControllerConfig FurutaController_DefaultConfig(void)
{
  FurutaControllerConfig config;

  config.sample_time_s = 0.0001f;
  config.pendulum_counts_per_rev = 4096.0f;
  config.motor_counts_per_rev = 44.0f;
  config.motor_gear_ratio = 9.267f;
  config.arm_outer_loop_divider = 5U;
  config.control_limit = 0.088f;
  config.deadband = 0.0f;

  config.pendulum_pid.kp = 60.0f;
  config.pendulum_pid.ki = 0.0f;
  config.pendulum_pid.kd = 2.0f;
  config.pendulum_pid.derivative_filter_n = 50.0f;
  config.pendulum_pid.prev_error = 0.0f;
  config.pendulum_pid.integral = 0.0f;
  config.pendulum_pid.derivative_state = 0.0f;
  config.pendulum_pid.output_min = -0.088f;
  config.pendulum_pid.output_max = 0.088f;

  config.arm_pid.kp = 0.05f;
  config.arm_pid.ki = 0.0f;
  config.arm_pid.kd = 0.08f;
  config.arm_pid.derivative_filter_n = 8.0f;
  config.arm_pid.prev_error = 0.0f;
  config.arm_pid.integral = 0.0f;
  config.arm_pid.derivative_state = 0.0f;
  config.arm_pid.output_min = -0.1f;
  config.arm_pid.output_max = 0.1f;


  return config;
}

void FurutaController_Init(FurutaController *controller,
                           const FurutaControllerConfig *config,
                           uint16_t pendulum_counter_now,
                           uint16_t motor_counter_now)
{
  if (controller == NULL || config == NULL)
  {
    return;
  }

  controller->config = *config;

  controller->measured.pendulum_angle_rad = 0.0f;
  controller->measured.pendulum_velocity_rad_s = 0.0f;
  controller->measured.arm_angle_rad = 0.0f;
  controller->measured.arm_velocity_rad_s = 0.0f;

  controller->pendulum_counts_total = 0;
  controller->pendulum_down_counts = 0;
  controller->pendulum_counter_prev = pendulum_counter_now;

  controller->arm_counts_total = 0;
  controller->motor_counter_prev = motor_counter_now;
  controller->arm_reference_rad = 0.0f;
  controller->arm_outer_loop_counter = 0U;

  controller->arm_hold_angle_rad = 0.0f;
  controller->control_enabled = 0U;

}

float FurutaController_WrapAngleRad(float angle_rad)
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

void FurutaController_UpdateMeasuredState(FurutaController *controller,
                                          uint16_t pendulum_counter_now,
                                          uint16_t motor_counter_now)
{
  int16_t pendulum_delta_counts;
  int16_t motor_delta_counts;
  float dt_s;
  float pendulum_angle_continuous_rad;
  const float two_pi = 6.28318530717958647692f;

  if (controller == NULL)
  {
    return;
  }

  dt_s = controller->config.sample_time_s;

  /* -------- 小摆杆：保持你原来的连续累计 + wrap 逻辑 -------- */
  pendulum_delta_counts = (int16_t)(pendulum_counter_now - controller->pendulum_counter_prev);
  controller->pendulum_counter_prev = pendulum_counter_now;
  controller->pendulum_counts_total += pendulum_delta_counts;

  pendulum_angle_continuous_rad =
      ((float)(controller->pendulum_counts_total - controller->pendulum_down_counts)
       / controller->config.pendulum_counts_per_rev) * two_pi;

  controller->measured.pendulum_angle_rad =
      FurutaController_WrapAngleRad(pendulum_angle_continuous_rad - 3.14159265358979323846f);

  /* -------- 大臂：改成“增量累加成连续位置”，不做 wrap -------- */
  motor_delta_counts = (int16_t)(motor_counter_now - controller->motor_counter_prev);
  controller->motor_counter_prev = motor_counter_now;
  controller->arm_counts_total += motor_delta_counts;

  controller->measured.arm_angle_rad =
      ((float)controller->arm_counts_total /
       (controller->config.motor_counts_per_rev * controller->config.motor_gear_ratio)) * two_pi;

  if (dt_s > 0.0f)
  {
    controller->measured.pendulum_velocity_rad_s =
        ((float)pendulum_delta_counts / controller->config.pendulum_counts_per_rev) * two_pi / dt_s;

    controller->measured.arm_velocity_rad_s =
        ((float)motor_delta_counts /
         (controller->config.motor_counts_per_rev * controller->config.motor_gear_ratio)) * two_pi / dt_s;
  }
  else
  {
    controller->measured.pendulum_velocity_rad_s = 0.0f;
    controller->measured.arm_velocity_rad_s = 0.0f;
  }
}

void FurutaController_EnableControl(FurutaController *controller)
{
  float pendulum_error;

  if (controller == NULL)
  {
    return;
  }

  controller->arm_hold_angle_rad = controller->measured.arm_angle_rad;
  controller->arm_reference_rad = 0.0f;
  controller->arm_outer_loop_counter = 0U;

  pendulum_error = -controller->measured.pendulum_angle_rad;

  controller->config.pendulum_pid.prev_error = pendulum_error;
  controller->config.pendulum_pid.integral = 0.0f;
  controller->config.pendulum_pid.derivative_state = 0.0f;

  controller->config.arm_pid.prev_error = 0.0f;
  controller->config.arm_pid.integral = 0.0f;
  controller->config.arm_pid.derivative_state = 0.0f;

  controller->control_enabled = 1U;
}
void FurutaController_DisableControl(FurutaController *controller)
{
  if (controller == NULL)
  {
    return;
  }

  controller->config.pendulum_pid.prev_error = 0.0f;
  controller->config.pendulum_pid.integral = 0.0f;
  controller->config.pendulum_pid.derivative_state = 0.0f;

  controller->config.arm_pid.prev_error = 0.0f;
  controller->config.arm_pid.integral = 0.0f;
  controller->config.arm_pid.derivative_state = 0.0f;

  controller->control_enabled = 0U;
}

FurutaControlOutput FurutaController_Step(FurutaController *controller,
                                          uint16_t pendulum_counter_now,
                                          uint16_t motor_counter_now)
{
  FurutaControlOutput output;
  float abs_control;

  output.raw_control = 0.0f;
  output.saturated_control = 0.0f;
  output.pwm_duty = 0.0f;
  output.direction = 0;

  if (controller == NULL)
  {
    return output;
  }

  FurutaController_UpdateMeasuredState(controller, pendulum_counter_now, motor_counter_now);
  if ((controller->control_enabled != 0U) &&
      (fabsf(controller->measured.pendulum_angle_rad) > 1.0f))
  {
      /* 想更早停机就把 1.0f 改小；想更晚停机就把 1.0f 改大 */
      FurutaController_DisableControl(controller);
      return output;   /* output 现在本来就是全 0 */
  }
  if (controller->control_enabled == 0U)
  {
    return output;
  }

  output.raw_control = furuta_compute_raw_control(controller);


  output.saturated_control = furuta_clamp(output.raw_control,
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
    output.direction = (output.saturated_control > 0.0f) ? 1 : -1;
    output.pwm_duty = furuta_clamp(abs_control / controller->config.control_limit, 0.0f, 1.0f);
  }

  return output;
}
