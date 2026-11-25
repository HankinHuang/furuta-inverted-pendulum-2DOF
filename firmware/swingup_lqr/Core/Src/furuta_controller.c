#include "furuta_controller.h"

#include <math.h>
#include <stddef.h>

/* ====== 供 STM32CubeMonitor 监控使用的全局变量 ====== */
volatile float monitor_raw_pend_angle = 0.0f;
volatile float monitor_raw_arm_angle = 0.0f;

/* ====== 新增：位置一阶低通滤波变量 ====== */
static float prev_filtered_pend_cont = 0.0f;
static float prev_filtered_arm_cont = 0.0f;
static uint8_t is_pos_lpf_initialized = 0;
#define POS_LPF_ALPHA 0.9f  // 位置滤波系数 Alpha

/* ====== 新增：速度一阶低通滤波变量 ====== */
static float prev_filtered_pend_vel = 0.0f;
static float prev_filtered_arm_vel = 0.0f;
static uint8_t is_vel_lpf_initialized = 0;
#define VEL_LPF_ALPHA 0.0f // 速度独立滤波系数 Alpha (可自由调节)

/* ====== 新增：齿轮回差补偿变量 ====== */
static float load_arm_cont = 0.0f;
static uint8_t is_backlash_initialized = 0;
#define BACKLASH_RAD 0.092457f
/* ======================================= */

/* -------------------- 核心状态估计与基础配置 -------------------- */

FurutaControllerConfig FurutaController_DefaultConfig(void)
{
  FurutaControllerConfig config;
  uint32_t i;
  uint8_t *ptr = (uint8_t *)&config;

  /* 结构体全清零，防止未初始化成员留下随机值 */
  for (i = 0; i < sizeof(FurutaControllerConfig); i++)
  {
    ptr[i] = 0U;
  }

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

  controller->arm_hold_angle_rad = 0.0f;
  controller->control_enabled = 0U;

  /* ====== 新增：重置滤波器初始化标志 ====== */
  is_pos_lpf_initialized = 0;
  is_vel_lpf_initialized = 0;

  is_backlash_initialized = 0; // 新增：重置回差初始化标志
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
  const float two_pi = 6.28318530717958647692f;

  if (controller == NULL)
  {
    return;
  }

  dt_s = controller->config.sample_time_s;

    /* -------- 1. 计算当前原始的连续角度 (不进行 Wrap) -------- */
    pendulum_delta_counts = (int16_t)(pendulum_counter_now - controller->pendulum_counter_prev);
    controller->pendulum_counter_prev = pendulum_counter_now;
    controller->pendulum_counts_total += pendulum_delta_counts;

    float raw_pend_cont = ((float)(controller->pendulum_counts_total - controller->pendulum_down_counts)
                           / controller->config.pendulum_counts_per_rev) * two_pi;

    motor_delta_counts = (int16_t)(motor_counter_now - controller->motor_counter_prev);
    controller->motor_counter_prev = motor_counter_now;
    controller->arm_counts_total += motor_delta_counts;

    float raw_arm_cont = ((float)controller->arm_counts_total /
                          (controller->config.motor_counts_per_rev * controller->config.motor_gear_ratio)) * two_pi;

    /* ====== 新增：齿轮回差滞回模型 ====== */
    if (is_backlash_initialized == 0)
    {
        load_arm_cont = raw_arm_cont;
        is_backlash_initialized = 1;
    }
    else
    {
        float half_backlash = BACKLASH_RAD / 2.1f;
        if (raw_arm_cont > load_arm_cont + half_backlash)
        {
            load_arm_cont = raw_arm_cont - half_backlash; // 正向啮合
        }
        else if (raw_arm_cont < load_arm_cont - half_backlash)
        {
            load_arm_cont = raw_arm_cont + half_backlash; // 反向啮合
        }
        // 在间隙内时，load_arm_cont 保持不变
    }

    /* ====== 修改：将补偿后的负载位置导出到全局变量 ====== */
    monitor_raw_pend_angle = raw_pend_cont;
    monitor_raw_arm_angle  = load_arm_cont;

    /* -------- 2. 位置一阶低通滤波 -------- */
    float current_filtered_pend;
    float current_filtered_arm;

    if (is_pos_lpf_initialized == 0)
    {
        current_filtered_pend = raw_pend_cont;
        current_filtered_arm  = load_arm_cont; // 使用补偿后的负载位置
        is_pos_lpf_initialized = 1;
    }
    else
    {
        current_filtered_pend = POS_LPF_ALPHA * prev_filtered_pend_cont + (1.0f - POS_LPF_ALPHA) * raw_pend_cont;
        current_filtered_arm  = POS_LPF_ALPHA * prev_filtered_arm_cont  + (1.0f - POS_LPF_ALPHA) * load_arm_cont; // 使用补偿后的负载位置
    }

    /* -------- 3. 将滤波后的位置赋值给状态 (摆杆进行 Wrap) -------- */
    controller->measured.pendulum_angle_rad = FurutaController_WrapAngleRad(current_filtered_pend - 3.14159265358979323846f);
    controller->measured.arm_angle_rad = current_filtered_arm;

    /* -------- 4. 计算速度 (先差分，后进行独立的一阶低通滤波) -------- */
      if (dt_s > 0.0f)
      {
          // 4.1 先通过滤波后的位置求得原始差分速度
          float raw_pend_vel = (current_filtered_pend - prev_filtered_pend_cont) / dt_s;
          float raw_arm_vel  = (current_filtered_arm - prev_filtered_arm_cont) / dt_s;

          // 4.2 对速度进行独立的一阶低通滤波
          if (is_vel_lpf_initialized == 0)
          {
              // 初始化第一帧，直接赋值防止阶跃
              controller->measured.pendulum_velocity_rad_s = raw_pend_vel;
              controller->measured.arm_velocity_rad_s = raw_arm_vel;
              is_vel_lpf_initialized = 1;
          }
          else
          {
              // 公式: V_n = Alpha * V_{n-1} + (1 - Alpha) * V_raw
              controller->measured.pendulum_velocity_rad_s =
                  VEL_LPF_ALPHA * prev_filtered_pend_vel + (1.0f - VEL_LPF_ALPHA) * raw_pend_vel;
              controller->measured.arm_velocity_rad_s =
                  VEL_LPF_ALPHA * prev_filtered_arm_vel + (1.0f - VEL_LPF_ALPHA) * raw_arm_vel;
          }

          // 4.3 记录当前速度供下一周期滤波使用
          prev_filtered_pend_vel = controller->measured.pendulum_velocity_rad_s;
          prev_filtered_arm_vel  = controller->measured.arm_velocity_rad_s;
      }
      else
      {
          controller->measured.pendulum_velocity_rad_s = 0.0f;
          controller->measured.arm_velocity_rad_s = 0.0f;
      }

    /* -------- 5. 保存当前滤波位置供下一周期使用 -------- */
    prev_filtered_pend_cont = current_filtered_pend;
    prev_filtered_arm_cont  = current_filtered_arm;
  }

void FurutaController_EnableControl(FurutaController *controller)
{
  if (controller == NULL)
  {
    return;
  }

  /* Store the arm angle at the switching instant and use it as the local
   * reference for subsequent LQR regulation. This prevents the controller from
   * reacting directly to the accumulated continuous arm angle after transfer. */
  controller->arm_hold_angle_rad = controller->measured.arm_angle_rad;
  controller->control_enabled = 1U;
}

void FurutaController_DisableControl(FurutaController *controller)
{
  if (controller == NULL)
  {
    return;
  }

  controller->control_enabled = 0U;
}
