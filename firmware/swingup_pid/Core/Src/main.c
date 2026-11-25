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
volatile uint8_t test_trigger = 0U;
float test_pwm_duty = 0.60f;
volatile uint32_t last_button_press = 0;
volatile uint8_t live_trigger_blue_btn = 0;

volatile int32_t motor_ticks_debug = 0;

FurutaController g_controller;
FurutaControllerConfig g_config;
FurutaControlOutput g_output;

SwingUpController g_swingup_controller;

uint32_t last_control_tick_ms = 0;
uint32_t last_print_tick_ms = 0;
volatile int16_t kickstart_countdown_ms = 0;
volatile uint8_t switch_flag = 0U;
volatile uint8_t pid_stop_latched = 0U;
volatile uint16_t switch_ready_count = 0U;


#define SWITCH_ANGLE_RAD            (0.15f)
#define SWITCH_PEND_VEL_RAD_S       (5.00f)
#define SWITCH_ARM_VEL_RAD_S        (5.0f)
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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart_print(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if 0
	if (GPIO_Pin == GPIO_PIN_13)
	{
		// Get the current system time in milliseconds
		        uint32_t current_time = HAL_GetTick();

		        // 2. Debounce: Only accept presses that are at least 200ms apart
		        if ((current_time - last_button_press) > 200)
		        {
		            // Toggle the variable cleanly
		            test_trigger = !test_trigger;

		            // Update the time for the next press
		            last_button_press = current_time;
		        }
	}

#endif
    if (GPIO_Pin == GPIO_PIN_13)
    {
        /* ---------- blue button: if PID is active, stop immediately ---------- */
        if ((switch_flag == 1U) && (g_controller.control_enabled != 0U))
        {
            Disable_SwingUp(&g_swingup_controller);
            FurutaController_DisableControl(&g_controller);

            motor_drive_apply(0, 0.0f);

            g_swingup_controller.output.raw_control = 0.0f;
            g_swingup_controller.output.saturated_control = 0.0f;
            g_swingup_controller.output.pwm_duty = 0.0f;
            g_swingup_controller.output.direction = 0;
            g_swingup_controller.logic.energy_control = 0.0f;
            g_swingup_controller.logic.torque = 0.0f;

            g_output.raw_control = 0.0f;
            g_output.saturated_control = 0.0f;
            g_output.pwm_duty = 0.0f;
            g_output.direction = 0;

            switch_flag = 0U;
            switch_ready_count = 0U;
            kickstart_countdown_ms = 0;
            pid_stop_latched = 1U;

            return;
        }

        /* ---------- blue button: if swing up is active, stop immediately ---------- */
        if ((switch_flag == 0U) && (g_swingup_controller.control_enabled != 0U))
        {
            Disable_SwingUp(&g_swingup_controller);
            FurutaController_DisableControl(&g_controller);

            motor_drive_apply(0, 0.0f);

            g_swingup_controller.output.raw_control = 0.0f;
            g_swingup_controller.output.saturated_control = 0.0f;
            g_swingup_controller.output.pwm_duty = 0.0f;
            g_swingup_controller.output.direction = 0;
            g_swingup_controller.logic.energy_control = 0.0f;
            g_swingup_controller.logic.torque = 0.0f;

            g_output.raw_control = 0.0f;
            g_output.saturated_control = 0.0f;
            g_output.pwm_duty = 0.0f;
            g_output.direction = 0;

            switch_flag = 0U;
            switch_ready_count = 0U;
            kickstart_countdown_ms = 0;
            pid_stop_latched = 1U;

            return;
        }

        /* ---------- blue button: if everything is stopped, start swing-up ---------- */
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
            Enable_SwingUp(&g_swingup_controller);

            g_swingup_controller.output.raw_control = 0.0f;
            g_swingup_controller.output.saturated_control = 0.0f;
            g_swingup_controller.output.pwm_duty = 0.0f;
            g_swingup_controller.output.direction = 0;
            g_swingup_controller.logic.energy_control = 0.0f;
            g_swingup_controller.logic.torque = 0.0f;

            g_output.raw_control = 0.0f;
            g_output.saturated_control = 0.0f;
            g_output.pwm_duty = 0.0f;
            g_output.direction = 0;

            pid_stop_latched = 0U;
            switch_flag = 0U;
            switch_ready_count = 0U;
            kickstart_countdown_ms = 100;
        }
    }

}

static void motor_drive_apply(int8_t direction, float pwm_duty)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim8);
  uint32_t compare = 0U;

  if (pwm_duty < 0.0f)
  {
    pwm_duty = 0.0f;
  }
  if (pwm_duty > PWM_DUTY_SAFETY_CAP)
  {
    pwm_duty = PWM_DUTY_SAFETY_CAP;
  }

  compare = (uint32_t)(pwm_duty * (float)arr);

  if (direction > 0)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_SET);
  }
  else if (direction < 0)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_RESET);
    compare = 0U;
  }

  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, compare);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(500);
  uart_print("Booting Swing Up controller...\r\n");

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);   // 小摆杆编码器 D4/D5
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   // motor encoder D7/D8
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);         // PWMA = D9

  motor_drive_apply(0, 0.0f);

  g_swingup_controller.output.raw_control = 0.0f;
  g_swingup_controller.output.saturated_control = 0.0f;
  g_swingup_controller.output.pwm_duty = 0.0f;
  g_swingup_controller.output.direction = 0U;

  g_swingup_controller.config.sample_time_s = 0.001f;
  g_swingup_controller.config.pendulum_counts_per_rev = 4096.0f;
  g_swingup_controller.config.motor_counts_per_rev = 44.0f;
  g_swingup_controller.config.motor_gear_ratio = 9.267f;
  //g_swingup_controller.config.arm_outer_loop_divider = 5U;
  g_swingup_controller.config.control_limit = 0.170f; // 0.080f 0.160f
  g_swingup_controller.config.deadband = 0.0f;
  g_swingup_controller.config.kickstart_pwm_duty = 0.10f;   // 原来 0.08

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

  g_swingup_controller.tuning_params.K = 0.0035f;           // 原来 0.0030
  g_swingup_controller.tuning_params.arm_kp = 1.00f;
  g_swingup_controller.tuning_params.arm_kd = 1.40f;       // 原来 0
  g_swingup_controller.tuning_params.target_energy_offset = 0.03f;
  g_swingup_controller.tuning_params.tanh_smoothing_factor = 100.0f;
  g_swingup_controller.tuning_params.v_theta_1_LPF_gain = 0.90f;
  g_swingup_controller.tuning_params.v_theta_arm_LPF_gain = 0.90f;

  g_config = FurutaController_DefaultConfig();
  g_config.motor_counts_per_rev = 44.0f;
  g_config.motor_gear_ratio = 9.267f;
  g_config.sample_time_s = 0.001f;
  g_config.control_limit = 1.0f;
  g_config.deadband = 0.0f;

  g_config.pendulum_pid.kp = 3.2f;
  g_config.pendulum_pid.ki = 0.0f;
  g_config.pendulum_pid.kd = 0.15f;
  g_config.pendulum_pid.output_min = -100.0f;
  g_config.pendulum_pid.output_max = 100.0f;
  g_config.pendulum_pid.derivative_filter_n = 10.0f;

  g_config.arm_outer_loop_divider = 7U;
  g_config.arm_pid.kp = 0.01f;
  g_config.arm_pid.ki = 0.0f;
  g_config.arm_pid.kd = 0.045f;
  g_config.arm_pid.output_min = -0.10f;
  g_config.arm_pid.output_max = 0.10f;
  g_config.arm_pid.derivative_filter_n = 8.0f;
  {
      uint16_t motor_counter_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
      motor_ticks_debug = (int32_t)motor_counter_now;

      SwingUp_Controller_init(&g_swingup_controller,
                            (uint16_t)__HAL_TIM_GET_COUNTER(&htim3),
                            motor_counter_now);

      FurutaController_Init(&g_controller,
                                  &g_config,
                                  (uint16_t)__HAL_TIM_GET_COUNTER(&htim3),
                                  motor_counter_now);

  }


  last_control_tick_ms = HAL_GetTick();
  last_print_tick_ms = HAL_GetTick();


  //uart_print("Controller initialized in SAFE mode. Pendulum down = reference, motor output disabled.\r\n");
  //uart_print("Manually swing the pendulum, then press the blue button to enable balance control.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#if 0
  while (1)
  {
	  if (test_trigger == 1U)
	  {
		  motor_drive_apply(1U, test_pwm_duty);
	  }
	  else
	  {
		  motor_drive_apply(0, 0.0f);
	  }
  }

#endif
  while (1)
  {
	  if (live_trigger_blue_btn == 1)
	  {
		live_trigger_blue_btn = 0; // Reset it immediately
		HAL_GPIO_EXTI_Callback(GPIO_PIN_13); // Simulate the interrupt
	  }

      uint32_t now_ms = HAL_GetTick();

      if ((now_ms - last_control_tick_ms) >= 1U)
      {
          uint16_t motor_counter_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
          uint16_t pendulum_cnt_now  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

          motor_ticks_debug = (int32_t)motor_counter_now;

          if ((switch_flag == 0U) && (pid_stop_latched == 0U))
          {
              g_swingup_controller.output = Compute_SwingUpOutput(&g_swingup_controller,
                                               pendulum_cnt_now,
                                               motor_counter_now);

              SwingUp_arm_angle_velocity_limit(&g_swingup_controller, 3.13f, 180.0f);
          }
          else
          {
              g_swingup_controller.output.raw_control = 0.0f;
              g_swingup_controller.output.saturated_control = 0.0f;
              g_swingup_controller.output.pwm_duty = 0.0f;
              g_swingup_controller.output.direction = 0;
              g_swingup_controller.logic.energy_control = 0.0f;
              g_swingup_controller.logic.torque = 0.0f;
          }

          g_output = FurutaController_Step(&g_controller,
                                           pendulum_cnt_now,
                                           motor_counter_now);
          /* ---------- swing-up -> PID switching ---------- */
          if ((g_swingup_controller.control_enabled != 0U) &&
              (switch_flag == 0U) &&
              (pid_stop_latched == 0U))
          {
              uint8_t switch_ready = 0U;
              float pa = g_controller.measured.pendulum_angle_rad;
              float pv = g_controller.measured.pendulum_velocity_rad_s;
              float av = g_controller.measured.arm_velocity_rad_s;

              if ((fabsf(pa) < SWITCH_ANGLE_RAD) &&
                  (fabsf(pv) < SWITCH_PEND_VEL_RAD_S) &&
                  (fabsf(av) < SWITCH_ARM_VEL_RAD_S))
              {
                  switch_ready = 1U;
              }

              if (switch_ready != 0U)
              {
                  if (switch_ready_count < SWITCH_CONFIRM_COUNT)
                  {
                      switch_ready_count++;
                  }
              }
              else
              {
                  switch_ready_count = 0U;
              }

              if (switch_ready_count >= SWITCH_CONFIRM_COUNT)
              {
                  Disable_SwingUp(&g_swingup_controller);

                  g_swingup_controller.output.raw_control = 0.0f;
                  g_swingup_controller.output.saturated_control = 0.0f;
                  g_swingup_controller.output.pwm_duty = 0.0f;
                  g_swingup_controller.output.direction = 0;
                  g_swingup_controller.logic.energy_control = 0.0f;
                  g_swingup_controller.logic.torque = 0.0f;

                  FurutaController_EnableControl(&g_controller);

                  switch_flag = 1U;
                  switch_ready_count = 0U;
                  kickstart_countdown_ms = 0;
                  pid_stop_latched = 0U;

                  uart_print("Switched to PID control.\r\n");
              }
          }
          else
          {
              switch_ready_count = 0U;
          }

          /* ---------- if PID loses balance, stop and wait for next blue button ---------- */
          if ((switch_flag == 1U) &&
              (g_controller.control_enabled == 0U) &&
              (pid_stop_latched == 0U))
          {
              Disable_SwingUp(&g_swingup_controller);
              FurutaController_DisableControl(&g_controller);

              g_swingup_controller.output.raw_control = 0.0f;
              g_swingup_controller.output.saturated_control = 0.0f;
              g_swingup_controller.output.pwm_duty = 0.0f;
              g_swingup_controller.output.direction = 0;
              g_swingup_controller.logic.energy_control = 0.0f;
              g_swingup_controller.logic.torque = 0.0f;

              switch_ready_count = 0U;
              kickstart_countdown_ms = 0;
              pid_stop_latched = 1U;

              motor_drive_apply(0, 0.0f);
              uart_print("PID failed, motor stopped. Press blue button to restart.\r\n");
          }
          /* ---------- apply motor output ---------- */
                  if (pid_stop_latched != 0U)
                  {
                      motor_drive_apply(0, 0.0f);
                  }
                  else if ((switch_flag == 0U) && (kickstart_countdown_ms > 0))
                  {
                      motor_drive_apply(1, 0.10f);
                      kickstart_countdown_ms--;
                  }
                  else if (switch_flag == 1U)
                  {
                      motor_drive_apply(g_output.direction, g_output.pwm_duty);
                  }
                  else
                  {
                      motor_drive_apply(g_swingup_controller.output.direction,
                                        g_swingup_controller.output.pwm_duty);
                  }

                  last_control_tick_ms = now_ms;
              }

      if ((now_ms - last_print_tick_ms) >= 50U)
      {
          char msg[260];

          float pa = g_swingup_controller.measured.pendulum_angle_rad;
          float pv = g_swingup_controller.measured.pendulum_velocity_rad_s;
          float av = g_swingup_controller.measured.arm_velocity_rad_s;

          uint8_t c1 = (fabsf(pa) < SWITCH_ANGLE_RAD) ? 1U : 0U;
          uint8_t c2 = (fabsf(pv) < SWITCH_PEND_VEL_RAD_S) ? 1U : 0U;
          uint8_t c3 = (fabsf(av) < SWITCH_ARM_VEL_RAD_S) ? 1U : 0U;

          snprintf(msg, sizeof(msg),
                   "sw=%u rc=%u suc=%u pa=%.3f pv=%.3f av=%.3f c=%u%u%u\r\n",
                   switch_flag,
                   switch_ready_count,
                   g_swingup_controller.swingup_success,
                   pa, pv, av,
                   c1, c2, c3);

          last_print_tick_ms = now_ms;
      }
          }

  /* USER CODE END 3 */
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
