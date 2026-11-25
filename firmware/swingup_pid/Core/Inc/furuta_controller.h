#ifndef FURUTA_CONTROLLER_H
#define FURUTA_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------- 控制器内部使用的数据结构 -------------------- */

/* 单个离散 PID/PD 控制器的内部状态。
 * 模型里 I 增益为 0，因此实际运行形态是 PD。
 * 这里仍然保留积分状态，便于后续继续调参或切回完整 PID。 */
typedef struct
{
  float kp;
  float ki;
  float kd;
  float derivative_filter_n;

  /* 上一周期误差*/
  float prev_error;

  /* 积分累加值*/
  float integral;
  float derivative_state;
  float output_min;
  float output_max;
} FurutaPidState;

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
  uint8_t arm_outer_loop_divider;
  /* 最终控制输出的对称限幅，例如 1.0 表示控制量限制在 [-1, 1] */
  float control_limit;

  /* 很小的输出直接归零，避免零点附近抖动 */
  float deadband;

  /* 摆杆角控制器，对应 Simulink 的 “PID subsystem pendulum angle” */
  FurutaPidState pendulum_pid;

  /* 摆臂角控制器，对应 Simulink 的 “PID subsystem arm angle” */
  FurutaPidState arm_pid;

} FurutaControllerConfig;

/* 从编码器估算出来的系统状态。
 * 这组量相当于原 Simulink 模型中用于反馈控制的观测状态。 */
typedef struct
{
  float pendulum_angle_rad;
  float pendulum_velocity_rad_s;
  float arm_angle_rad;
  float arm_velocity_rad_s;
} FurutaMeasuredState;

/* 控制器运行时上下文。
 * 该结构保存编码器上一次计数、累计摆杆圈数、两路 PID 内部状态等。 */
typedef struct
{
  FurutaControllerConfig config;
  FurutaMeasuredState measured;

  /* 摆杆累计计数，用于从 16 位编码器计数器扩展成连续角度 */
  int32_t pendulum_counts_total;

  /* 启动时自然下垂位置对应的累计计数。
   * 之后摆杆角度按“相对下垂位置再减 pi”计算，
   * 因而 upright = 0 rad，downward ≈ -pi rad。 */
  int32_t pendulum_down_counts;

  /* 上一次采样时 TIM3 的原始计数 */
  uint16_t pendulum_counter_prev;

  /* 大臂累计计数。
   * 和成功案例的 Location 一样，始终按增量累加，不做 wrap。 */
  int32_t arm_counts_total;

  /* 上一次采样时 TIM1 的原始 16 位计数器值 */
  uint16_t motor_counter_prev;

  /* 外环最近一次计算得到的摆杆角参考 */
  float arm_reference_rad;

  /* 外环分频计数器 */
  uint8_t arm_outer_loop_counter;

  /* 控制启用瞬间的大臂保持角参考。
   * 这样你手动起摆到合适位置按按钮后，
   * 大臂不会先强行回到上电初始角。 */
  float arm_hold_angle_rad;

  /* 0 = 只更新状态不输出；1 = 允许控制器输出 */
  uint8_t control_enabled;
} FurutaController;

/* 控制器输出结果。
 * raw_control 是控制律原始结果；
 * saturated_control 是限幅后的结果；
 * pwm_duty 是映射到 [0,1] 的占空比；
 * direction 表示方向：1 正转，-1 反转，0 停止。 */
typedef struct
{
  float raw_control;
  float saturated_control;
  float pwm_duty;
  int8_t direction;
} FurutaControlOutput;

/* -------------------- 对外接口 -------------------- */

/* 创建一份与 Simulink 部署模型一致的默认配置。
 * 当前参数来源：
 * 1. PId_deploy_hw.slx / system_29.xml
 * 2. PId_deploy_hw.slx / system_107.xml
 * 3. functions/pid_hw.m 中的 Ts
 */
FurutaControllerConfig FurutaController_DefaultConfig(void);

/* 初始化控制器上下文。
 * 需要把编码器当前值传进来，避免第一拍差分时出现错误的大速度尖峰。 */
extern void FurutaController_Init(FurutaController *controller,
                           const FurutaControllerConfig *config,
                           uint16_t pendulum_counter_now,
                           uint16_t motor_counter_now);

void FurutaController_UpdateMeasuredState(FurutaController *controller,
                                          uint16_t pendulum_counter_now,
                                          uint16_t motor_counter_now);

FurutaControlOutput FurutaController_Step(FurutaController *controller,
                                          uint16_t pendulum_counter_now,
                                          uint16_t motor_counter_now);

/* 把任意角度折返到 [-pi, pi)。 */
float FurutaController_WrapAngleRad(float angle_rad);



/* 启用闭环控制。
 * 建议流程：
 * 1. 上电时让摆杆自然下垂
 * 2. 程序先只读状态，不输出电机
 * 3. 你手动起摆
 * 4. 靠近 upright 时按按钮，调用本函数开始平衡控制 */
void FurutaController_EnableControl(FurutaController *controller);

/* 关闭闭环控制并清空内部 PID 状态。 */
void FurutaController_DisableControl(FurutaController *controller);



#ifdef __cplusplus
}
#endif

#endif /* FURUTA_CONTROLLER_H */
