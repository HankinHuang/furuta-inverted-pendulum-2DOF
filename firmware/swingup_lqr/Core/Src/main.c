/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "furuta_controller.h"
#include "furuta_swingup.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_PERIOD_MS        (1U)
#define TELEMETRY_PERIOD_MS      (50U)
#define PWM_DUTY_SAFETY_CAP      (0.80f)
#define MAX_VOLTAGE              (12.0f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile int32_t motor_ticks_debug = 0;
volatile uint8_t live_trigger_blue_btn = 0; // 新增：用于 IDE 内部触发蓝键中断
volatile float debug_unfiltered_u = 0.0f;

FurutaController g_controller;
FurutaControllerConfig g_config;
FurutaControlOutput g_output;

SwingUpController g_swingup_controller;

uint32_t last_control_tick_ms = 0;
uint32_t last_print_tick_ms = 0;
volatile int16_t kickstart_countdown_ms = 0;
volatile uint8_t switch_flag = 0U;
volatile uint8_t lqr_stop_latched = 0U;
volatile uint16_t switch_ready_count = 0U;


// ================= LQR 增益调度 (Soft-LQR) 参数 =================
// K 矩阵:[臂角, 臂速, 摆角, 摆速] [arm_angle, arm_vel, pend_angle, pend_vel]

// 1. 内圈 (极小角度，< 2度)：极度轻柔，大幅减小 K4 消除稳态震荡
float K[4] = {-0.4219f, -1.2111f, -17.5453f, -1.3306f};

/* ====== LQR 输出低通滤波器参数 ====== */
static float lqr_u_filtered = 0.71f; // 保存上一周期的滤波输出
#define LQR_LPF_ALPHA  (0.29f)       // 滤波系数 Alpha。取值范围 0.0 ~ 1.0

/* ---------- switch 逻辑 ---------- */
#define SWITCH_ANGLE_RAD            (0.25f)
#define SWITCH_PEND_VEL_RAD_S       (5.00f)
#define SWITCH_ARM_VEL_RAD_S        (5.00f)
#define SWITCH_CONFIRM_COUNT        (10U)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
static void uart_print(const char *s);
static void motor_drive_apply(int8_t direction, float pwm_duty);
static void zero_swingup_runtime(void);
static void zero_lqr_output(void);
static FurutaControlOutput lqr_compute_output(const FurutaController *controller);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 1. 串口打印函数
 * 作用：将字符串发送到电脑或其他串口终端，通常用于输出调试信息。*/
static void uart_print(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

/* * 2. 起摆控制器状态清零函数
 * 作用：将起摆（Swing-up）控制器的所有输出值强制设为 0。
 * 这在系统启动、急停或切换到 LQR 模式时调用，防止上一阶段的残留数值导致电机突然乱转。 */
static void zero_swingup_runtime(void)
{
  g_swingup_controller.output.raw_control = 0.0f;			// 清除原始控制量
  g_swingup_controller.output.saturated_control = 0.0f;		// 清除限幅后的控制量
  g_swingup_controller.output.pwm_duty = 0.0f;				// 清除 PWM 占空比（电机转速控制值）
  g_swingup_controller.output.direction = 0;				// 清除电机方向
  g_swingup_controller.logic.energy_control = 0.0f;			// 清除能量控制算法的历史数据
  g_swingup_controller.logic.torque = 0.0f;					// 清除计算出的目标扭矩
}

/* * 3. LQR 控制器输出清零函数
 * 作用：将 LQR（线性二次型调节器）的全局输出变量强制设为 0。
 * 用途与 zero_swingup_runtime 类似，用于停止平衡控制时安全切断电机动力。*/
static void zero_lqr_output(void)
{
  g_output.raw_control = 0.0f;				// 清除原始控制量
  g_output.saturated_control = 0.0f;		// 清除限幅后的控制量
  g_output.pwm_duty = 0.0f;					// 清除 PWM 占空比
  g_output.direction = 0;					// 清除电机方向
  /* ====== 新增：状态重置时清空历史滤波值 ====== */
  lqr_u_filtered = 0.0f;
}

/* * 4. LQR 控制量计算函数（部分）
 * 作用：读取当前倒立摆的状态（角度、速度），通过矩阵运算计算出电机应该输出多大的力。
 * 此段代码为该函数的“初始化与安全检查”部分。 */
static FurutaControlOutput lqr_compute_output(const FurutaController *controller)
{
  FurutaControlOutput output;				// 定义一个局部变量，用于存储即将计算出的输出结果
  float u = 0.0f;							// 定义并初始化控制律变量 u
  float arm_angle;							// 定义旋臂角度变量
  float arm_vel;							// 定义旋臂角速度变量
  float pend_angle;							// 定义摆杆角度变量
  float pend_vel;							// 定义摆杆角速度变量

  // 赋初值：默认将输出参数全部置零
  output.raw_control = 0.0f;
  output.saturated_control = 0.0f;
  output.pwm_duty = 0.0f;
  output.direction = 0;

  // 安全防线：如果控制器指针为空，或者控制器未被启用
  if (controller == NULL || controller->control_enabled == 0U)	// 直接返回全为0的输出结果，中止后续计算，确保电机不会动作
  {
    return output;
  }

  /* ================= 1. 核心控制律计算 ================= */
  /* 以手臂相对于切换瞬间的角度作为LQR调节的参考，使控制器围绕实际转移位置而非累积的连续手臂角度进行平衡。 */
  arm_angle = FurutaController_WrapAngleRad(controller->measured.arm_angle_rad - controller->arm_hold_angle_rad);
  arm_vel = controller->measured.arm_velocity_rad_s;
  pend_angle = controller->measured.pendulum_angle_rad;
  pend_vel = controller->measured.pendulum_velocity_rad_s;

  /* ================= 2. 核心控制律计算 ================= */
  /* LQR: x =[arm_angle, arm_vel, pend_angle, pend_vel] */
  u = - K[0] * (- arm_angle)
    - K[1] * (- arm_vel)
    - K[2] * pend_angle
    - K[3] * pend_vel;

  debug_unfiltered_u = u;
  /* ==========================================================
  * 新增：一阶低通滤波 (Exponential Moving Average)
  * 公式：Y_n = Alpha * X_n + (1 - Alpha) * Y_{n-1}
  * ========================================================== */
  lqr_u_filtered = LQR_LPF_ALPHA * u + (1.0f - LQR_LPF_ALPHA) * lqr_u_filtered;
  u = lqr_u_filtered; // 用滤波后的平滑电压覆盖原本的 u，向下传递

  /* ================= 2. 死区补偿 (Deadzone Compensation) ================= */
  // 匹配 MATLAB 逻辑：剔除 (-0.6, 0.6) 区间的微小输出，消除稳态震荡
  float deadband_offset = 0.5f;

  if (u > deadband_offset)
  {   // 信号在[0.6, 12] 内，保持原值
      u = u;}
  else if (u < -deadband_offset)
  {   // 信号在[-12, -0.6] 内，保持原值
      u = u;}
  else
  {   // 信号在 (-0.6, 0.6) 之间，强制抹零断电
      u = 0.0f;}

  /* ================= 3. 电压限幅 (Saturation / Clamping) ================= */
  if (u > MAX_VOLTAGE) u = MAX_VOLTAGE;
  if (u < -MAX_VOLTAGE) u = -MAX_VOLTAGE;

  /* ================= 4. 输出变量赋值 ================= */
  output.raw_control = u;
  output.saturated_control = u;
  output.pwm_duty = fabsf(u) / MAX_VOLTAGE;

  /* ================= 5. 确定电机转动方向 ================= */
  if (u > 0.0f)
  {output.direction = 1;
  }
  else if (u < 0.0f)
  {output.direction = -1;
  }
  else
  {output.direction = 0;
  }

  return output;
}

/* * 1. 外部中断回调函数 (EXTI Callback)
 * 作用：处理按键逻辑，实现倒立摆系统的“紧急停止”和“一键启动”。*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)		//蓝色按键
    {
        /* ---------- blue button: if LQR is active, stop immediately ---------- */
    	/* ========== 逻辑 1：紧急停止 (LQR 阶段) ========== */
        if ((switch_flag == 1U) && (g_controller.control_enabled != 0U))
        {
            Disable_SwingUp(&g_swingup_controller);
            FurutaController_DisableControl(&g_controller);

            zero_swingup_runtime();
            zero_lqr_output();
            motor_drive_apply(0, 0.0f);

            switch_flag = 0U;
            switch_ready_count = 0U;
            kickstart_countdown_ms = 0;
            lqr_stop_latched = 1U;

            uart_print("LQR stopped by blue button.\r\n");
            return;
        }

        /* ========== 新增逻辑 1.5：紧急停止 (起摆阶段) ========== */
        if ((switch_flag == 0U) && (g_swingup_controller.control_enabled != 0U))
        {
            Disable_SwingUp(&g_swingup_controller);
            FurutaController_DisableControl(&g_controller);

            zero_swingup_runtime();
            zero_lqr_output();
            motor_drive_apply(0, 0.0f);

            switch_flag = 0U;
            switch_ready_count = 0U;
            kickstart_countdown_ms = 0;
            lqr_stop_latched = 1U; // 锁定状态，防止立即重启

            uart_print("Swing-up stopped by blue button.\r\n");
            return;
        }

        /* ---------- blue button: if everything is stopped, start swing-up ---------- */
        /* ========== 逻辑 2：启动起摆 (Swing-up) ========== */
        if ((g_swingup_controller.control_enabled == 0U) &&
            (g_controller.control_enabled == 0U))
        {
            uint16_t pendulum_cnt_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
            uint16_t motor_counter_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

            motor_drive_apply(0, 0.0f);

            FurutaController_DisableControl(&g_controller);
            SwingUp_Controller_init(&g_swingup_controller,
                                    pendulum_cnt_now,
                                    motor_counter_now);

            /* ====== 新增：同步初始化 LQR 控制器，统一零点 ====== */
            FurutaController_Init(&g_controller,
                                  &g_config,
                                  pendulum_cnt_now,
                                  motor_counter_now);

            Enable_SwingUp(&g_swingup_controller);

            zero_swingup_runtime();
            zero_lqr_output();

            lqr_stop_latched = 0U;
            switch_flag = 0U;
            switch_ready_count = 0U;
            kickstart_countdown_ms = 100;

            uart_print("Swing-up ENABLED by blue button.\r\n");
        }
    }
}

/* * 2. 电机底层驱动应用函数
 * 作用：将算法计算出的方向和 PWM 占空比，转化为 STM32 的引脚输出。
 * 参数：
 * direction: 电机转动方向 (>0 正转, <0 反转, 0 停止)
 * pwm_duty:  PWM 占空比 (0.0 到 1.0 的浮点数，代表输出功率百分比)
 */
static void motor_drive_apply(int8_t direction, float pwm_duty)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim8);
  uint32_t compare = 0U;

  /* ========== 1. 软件限幅保护 ========== */
  if (pwm_duty < 0.0f)
  {
    pwm_duty = 0.0f;
  }
  if (pwm_duty > PWM_DUTY_SAFETY_CAP)
  {
    pwm_duty = PWM_DUTY_SAFETY_CAP;
  }

  /* ========== 2. 占空比数值转换 ========== */
  compare = (uint32_t)(pwm_duty * (float)arr);

  /* ========== 3. 引脚方向控制 ========== */
  if (direction > 0)
  {	// 正转：PA10 输出低电平，PB3 输出高电平
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_SET);
  }
  else if (direction < 0)
  {	// 反转：PA10 输出高电平，PB3 输出低电平
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_RESET);
  }
  else
  {	// 停止：两个引脚均输出低电平（电机刹车或滑行，取决于电机驱动板设计）
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_RESET);
    compare = 0U;
  }

  /* ========== 4. 执行 PWM 输出 ========== */
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, compare);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
  uart_print("Booting Swing Up + LQR controller...\r\n");

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  motor_drive_apply(0, 0.0f);

  zero_swingup_runtime();
  zero_lqr_output();

  /* ---------- swing-up init 起摆参数 ---------- */
  // 硬件配置参数：采样时间、编码器线数、电机减速比、控制限幅等
  g_swingup_controller.config.sample_time_s = 0.001f;
  g_swingup_controller.config.pendulum_counts_per_rev = 4096.0f;
  g_swingup_controller.config.motor_counts_per_rev = 44.0f;
  g_swingup_controller.config.motor_gear_ratio = 9.267f;
  g_swingup_controller.config.control_limit = 0.150f;
  g_swingup_controller.config.deadband = 0.0f;
  g_swingup_controller.config.kickstart_pwm_duty = 0.10f;

  // 机械物理参数：旋臂长度 (L_arm)、惯量 (I_arm)、质量 (m_1)、重力加速度 (g) 及摩擦系数等
  g_swingup_controller.mech_params.L_arm = 145e-3f;
  g_swingup_controller.mech_params.I_arm = 7.1e-4f;
  g_swingup_controller.mech_params.lcom_1 = 0.0503f;
  g_swingup_controller.mech_params.I_1 = 1.05e-4f;
  g_swingup_controller.mech_params.m_1 = 0.0542f;
  g_swingup_controller.mech_params.g = 9.81f;

  g_swingup_controller.mech_params.f_v_arm = 0.0f;
  g_swingup_controller.mech_params.f_c_arm = 0.0f;
  g_swingup_controller.mech_params.k_fc_arm = 0.0f;
  g_swingup_controller.mech_params.f_v_1 = 0.0f;
  g_swingup_controller.mech_params.f_c_1 = 0.0f;
  g_swingup_controller.mech_params.k_fc_1 = 0.0f;

  // 算法整定参数：能量控制增益 (K)、旋臂位置 PD 参数 (arm_kp, arm_kd)、低通滤波增益等
  g_swingup_controller.tuning_params.K = 0.0035f;
  g_swingup_controller.tuning_params.arm_kp = 1.00f;
  g_swingup_controller.tuning_params.arm_kd = 1.40f;
  g_swingup_controller.tuning_params.target_energy_offset = 0.03f;
  g_swingup_controller.tuning_params.tanh_smoothing_factor = 100.0f;
  g_swingup_controller.tuning_params.v_theta_1_LPF_gain = 0.0f;
  g_swingup_controller.tuning_params.v_theta_arm_LPF_gain = 0.0f;

  /* ---------- LQR measured-state init LQR 测量状态初始化 ---------- */
  g_config = FurutaController_DefaultConfig();		// 加载默认配置模板
  g_config.pendulum_counts_per_rev = 4096.0f;		// 覆盖摆杆编码器参数
  g_config.motor_counts_per_rev = 44.0f;			// 覆盖电机编码器参数
  g_config.motor_gear_ratio = 9.267f;				// 覆盖齿轮减速比
  g_config.sample_time_s = 0.001f;					// 设定 LQR 采样周期为 1ms
  g_config.control_limit = MAX_VOLTAGE;				// 设定最大输出电压限制
  g_config.deadband = 0.0f;							// 设定死区参数

  {	  // 读取上电瞬间两个编码器的原始计数值
      uint16_t motor_counter_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
      uint16_t pendulum_cnt_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

      motor_ticks_debug = (int32_t)motor_counter_now;
      // 使用当前的物理位置作为零点（基准点），初始化起摆控制器
      SwingUp_Controller_init(&g_swingup_controller,
                              pendulum_cnt_now,
                              motor_counter_now);
      // 使用相同的物理位置，初始化 LQR 控制器
      FurutaController_Init(&g_controller,
                            &g_config,
                            pendulum_cnt_now,
                            motor_counter_now);
  }

  last_control_tick_ms = HAL_GetTick();
  last_print_tick_ms = HAL_GetTick();

  uart_print("Controller initialized in SAFE mode. Press blue button to start swing-up.\r\n");
  /* USER CODE END 2 */

  while (1)
  {
      // 新增：IDE 触发逻辑
      if (live_trigger_blue_btn == 1)
      {
          live_trigger_blue_btn = 0;
          HAL_GPIO_EXTI_Callback(GPIO_PIN_13);
      }

	  uint32_t now_ms = HAL_GetTick();		// 获取当前系统运行的毫秒数
      /* ================= 任务调度 1：高频控制循环 =================
      * 保证控制算法以恒定的周期执行（CONTROL_PERIOD_MS，通常为 1ms）。
      * 这对数字控制系统（如 PID、LQR）至关重要，时间间隔不准会导致系统发散。*/
      if ((now_ms - last_control_tick_ms) >= CONTROL_PERIOD_MS)
      {	  // 读取电机和摆杆当前位置的原始计数值
          uint16_t motor_counter_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
          uint16_t pendulum_cnt_now  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
          uint8_t switch_ready = 0U;

          motor_ticks_debug = (int32_t)motor_counter_now;

          /* ---------- swing-up output 起摆控制输出计算 ---------- */
          // 如果当前处于起摆模式（switch_flag == 0）且系统没有被紧急停止
          if ((switch_flag == 0U) && (lqr_stop_latched == 0U))
          {	  // 计算起摆所需的能量控制量
              g_swingup_controller.output = Compute_SwingUpOutput(&g_swingup_controller,
                                                                  pendulum_cnt_now,
                                                                  motor_counter_now);

              /* 限制旋臂的转动角度和速度，防止起摆时动作过大撞坏机械结构 */
              SwingUp_arm_angle_velocity_limit(&g_swingup_controller, 3.13f, 180.0f);
          }
          else
          {	  // 如果不在起摆模式，将起摆相关的输出变量清零，防止干扰
              zero_swingup_runtime();
          }

          /* Restore the PID-version switching flow:
           * evaluate the swing-up -> LQR handover using only the threshold-based
           * consecutive-sample check, with no swingup_success shortcut and no
           * LQR blend-in ramp after switching. */
          /* ---------- 状态更新与切换判断 ---------- */
          FurutaController_UpdateMeasuredState(&g_controller,
                                               pendulum_cnt_now,
                                               motor_counter_now);
          // 如果当前正在起摆阶段，判断是否满足切换到 LQR 倒立平衡的条件
          if ((g_swingup_controller.control_enabled != 0U) &&
              (switch_flag == 0U) &&
              (lqr_stop_latched == 0U))
          {
              float pa = g_controller.measured.pendulum_angle_rad;			// 摆杆角度
              float pv = g_controller.measured.pendulum_velocity_rad_s;		// 摆杆角速度
              float av = g_controller.measured.arm_velocity_rad_s;			// 旋臂角速度

              // 核心判断：如果摆杆接近正上方，且自身的转速足够慢，且电机旋臂也没转得太快
              if ((fabsf(pa) < SWITCH_ANGLE_RAD) &&
                  (fabsf(pv) < SWITCH_PEND_VEL_RAD_S) &&
                  (fabsf(av) < SWITCH_ARM_VEL_RAD_S))
              {
                  switch_ready = 1U;		// 满足单次条件
              }
              // 防抖机制：必须连续 N 次（SWITCH_CONFIRM_COUNT）满足条件，才确认切换。
              // 防止传感器偶尔的噪声导致误切。
              if (switch_ready != 0U)
              {
                  if (switch_ready_count < SWITCH_CONFIRM_COUNT)
                  {
                      switch_ready_count++;
                  }
              }
              else
              {
                  switch_ready_count = 0U;	// 一旦中断，计数器清零重新算
              }
              // 如果确认次数达标，执行模式切换动作
              if (switch_ready_count >= SWITCH_CONFIRM_COUNT)
              {
                  /* Clear swing-up runtime output before enabling LQR, so the
                   * handover starts from a clean zeroed transition state. */
                  Disable_SwingUp(&g_swingup_controller);
                  zero_swingup_runtime();

                  /* ====== 新增：将当前旋臂角度设定为 LQR 平衡的参考零点 ====== */
                  g_controller.arm_hold_angle_rad = g_controller.measured.arm_angle_rad;

                  FurutaController_EnableControl(&g_controller);
                  zero_lqr_output();

                  switch_flag = 1U;				// 标志位变为 1，表示正式进入 LQR 阶段
                  switch_ready_count = 0U;
                  kickstart_countdown_ms = 0;
                  lqr_stop_latched = 0U;

                  uart_print("Switched to LQR control.\r\n");
              }
          }
          else
          {
              switch_ready_count = 0U;
          }

          /* ---------- if LQR is active and loses balance, disable control ---------- */
          /* ---------- LQR 安全保护 ---------- */
          if ((switch_flag == 1U) &&
              (g_controller.control_enabled != 0U) &&
              (fabsf(g_controller.measured.pendulum_angle_rad) > 1.0f))
          {
              FurutaController_DisableControl(&g_controller);
          }

          /* Restore the PID-version handover style after switching:
           * apply full LQR output immediately, without any blend-in ramp. */
          /* ---------- LQR 输出计算 ---------- */
          if ((switch_flag == 1U) && (g_controller.control_enabled != 0U))
          {
              g_output = lqr_compute_output(&g_controller);
          }
          else
          {
              zero_lqr_output();
          }

          /* ---------- if LQR loses balance, stop and wait for next blue button ---------- */
          /* ---------- LQR 失败善后 ---------- */
          if ((switch_flag == 1U) &&
              (g_controller.control_enabled == 0U) &&
              (lqr_stop_latched == 0U))
          {
              Disable_SwingUp(&g_swingup_controller);
              FurutaController_DisableControl(&g_controller);

              zero_swingup_runtime();
              zero_lqr_output();

              switch_ready_count = 0U;
              kickstart_countdown_ms = 0;
              lqr_stop_latched = 1U;		// 锁定状态，必须按按键才能复位

              motor_drive_apply(0, 0.0f);	// 硬件断电
              uart_print("LQR failed, motor stopped. Press blue button to restart.\r\n");
          }

          /* ---------- apply motor output 输出汇总与电机驱动 ---------- */
          if (lqr_stop_latched != 0U)
          {	  // 状态 1：已锁定停机 -> 输出全 0
              motor_drive_apply(0, 0.0f);
          }
          else if ((switch_flag == 0U) && (kickstart_countdown_ms > 0))
          {	  // 状态 2：起步脉冲阶段 -> 强制电机以 30% 动力正转，持续短时间，用于打破静摩擦启动摆动
              motor_drive_apply(1, 0.30f);
              kickstart_countdown_ms--;
          }
          else if (switch_flag == 1U)
          {	  // 状态 3：LQR 平衡阶段 -> 将 LQR 算法的计算结果交给电机
              motor_drive_apply(g_output.direction, g_output.pwm_duty);
          }
          else
          {	  // 状态 4：正常起摆阶段 -> 将起摆算法的计算结果交给电机
              motor_drive_apply(g_swingup_controller.output.direction,
                                g_swingup_controller.output.pwm_duty);
          }

          last_control_tick_ms = now_ms;		// 更新控制时间戳
      }
      /* ================= 任务调度 2：低频打印循环 =================
      * 以 50ms (TELEMETRY_PERIOD_MS) 为周期向串口发送当前数据。
      * 为什么不放在 1ms 循环里？因为串口发送字符串很慢，会阻塞 CPU，导致控制周期不准。*/
      if ((now_ms - last_print_tick_ms) >= TELEMETRY_PERIOD_MS)
      {
          char msg[260];

          float pa = g_swingup_controller.measured.pendulum_angle_rad;
          float pv = g_swingup_controller.measured.pendulum_velocity_rad_s;
          float av = g_swingup_controller.measured.arm_velocity_rad_s;
          // 将是否达到切换条件的判断转化为 1 或 0
          uint8_t c1 = (fabsf(pa) < SWITCH_ANGLE_RAD) ? 1U : 0U;
          uint8_t c2 = (fabsf(pv) < SWITCH_PEND_VEL_RAD_S) ? 1U : 0U;
          uint8_t c3 = (fabsf(av) < SWITCH_ARM_VEL_RAD_S) ? 1U : 0U;
          // 格式化输出字符串，包含：状态标志、连续满足条件次数、三个物理量、条件布尔值、PWM输出等
          snprintf(msg, sizeof(msg),
                   "sw=%u rc=%u suc=%u pa=%.3f pv=%.3f av=%.3f c=%u%u%u lqr=%.2f\r\n",
                   switch_flag,
                   switch_ready_count,
                   g_swingup_controller.swingup_success,
                   pa, pv, av,
                   c1, c2, c3,
                   g_output.pwm_duty * (g_output.direction == 1 ? 1.0f :
                                        (g_output.direction == -1 ? -1.0f : 0.0f)));

          uart_print(msg);
          last_print_tick_ms = now_ms;
      }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 4199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
