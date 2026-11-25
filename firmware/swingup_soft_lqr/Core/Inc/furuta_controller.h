#ifndef FURUTA_CONTROLLER_H
#define FURUTA_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------- 基础配置 -------------------- */

typedef struct
{
  /* 控制周期，单位：秒 */
  float sample_time_s;

  /* 摆杆编码器每转脉冲数 */
  float pendulum_counts_per_rev;

  /* 电机编码器每转脉冲数 */
  float motor_counts_per_rev;

  /* 电机减速比 */
  float motor_gear_ratio;

  /* 预留的控制限幅参数 */
  float control_limit;

  /* 很小的输出可直接归零的死区 */
  float deadband;
} FurutaControllerConfig;

/* -------------------- 测量状态 -------------------- */

typedef struct
{
  float pendulum_angle_rad;
  float pendulum_velocity_rad_s;
  float arm_angle_rad;
  float arm_velocity_rad_s;
} FurutaMeasuredState;

/* -------------------- 控制器上下文 -------------------- */

typedef struct
{
  FurutaControllerConfig config;
  FurutaMeasuredState measured;

  /* 摆杆累计计数 */
  int32_t pendulum_counts_total;

  /* 下垂参考位置对应的累计计数 */
  int32_t pendulum_down_counts;

  /* 上一次采样的摆杆编码器原始计数 */
  uint16_t pendulum_counter_prev;

  /* 大臂累计计数 */
  int32_t arm_counts_total;

  /* 上一次采样的电机编码器原始计数 */
  uint16_t motor_counter_prev;

  /* 启用控制瞬间记录的大臂保持角 */
  float arm_hold_angle_rad;

  /* 控制使能标志：0=关闭，1=开启 */
  uint8_t control_enabled;
} FurutaController;

/* -------------------- 输出结构 -------------------- */

typedef struct
{
  float raw_control;
  float saturated_control;
  float pwm_duty;
  int8_t direction;
} FurutaControlOutput;

/* -------------------- 对外接口 -------------------- */

FurutaControllerConfig FurutaController_DefaultConfig(void);

void FurutaController_Init(FurutaController *controller,
                           const FurutaControllerConfig *config,
                           uint16_t pendulum_counter_now,
                           uint16_t motor_counter_now);

void FurutaController_UpdateMeasuredState(FurutaController *controller,
                                          uint16_t pendulum_counter_now,
                                          uint16_t motor_counter_now);

float FurutaController_WrapAngleRad(float angle_rad);

void FurutaController_EnableControl(FurutaController *controller);
void FurutaController_DisableControl(FurutaController *controller);

#ifdef __cplusplus
}
#endif

#endif /* FURUTA_CONTROLLER_H */
