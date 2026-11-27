# Swing-Up + Cascade PID Firmware (`STM32F446RE`)

Embedded C project for autonomous swing-up and dual-loop cascade PID stabilization on the STM32 Nucleo-F446RE platform (`Swingup_PID.ioc`).

## Control Architecture

1. **Swing-Up Stage (`Core/Src/furuta_swingup.c`)**:
   - Feedback-linearized Lyapunov energy controller with continuous $\tanh$ smoothing and filtered state feedback.
2. **Mode Switching (`Core/Src/main.c`)**:
   - Transitions from swing-up to PID balancing when $|\theta_p| < 0.15\text{ rad}$ (tuned up to $0.25\text{ rad}$ across tests), $|\dot{\theta}_p| < 5.0\text{ rad/s}$, and $|\dot{\theta}_a| < 5.0\text{ rad/s}$ hold for `10` consecutive control samples (`SWITCH_CONFIRM_COUNT = 10`).
3. **Dual-Loop Cascade PID (`Core/Src/furuta_controller.c`)**:
   - **Inner loop (pendulum angle)**: regulates upright pendulum angle around `0 rad` (`kp = 60.0`, `ki = 0.0`, `kd = 2.0`, `derivative_filter_n = 50.0` in `FurutaController_DefaultConfig`; see `main.c` for runtime overrides).
   - **Outer loop (rotary arm angle)**: runs at a divided outer-loop rate (`arm_outer_loop_divider`) to bound arm drift (`kp = 0.05`, `ki = 0.0`, `kd = 0.08`, `derivative_filter_n = 8.0`).
   - Output is clamped to `PWM_DUTY_SAFETY_CAP = 0.80` on `TIM8_CH2`.

## Build & Flash

Import this folder into **STM32CubeIDE**, build the target, and flash via onboard ST-LINK. Press the blue user button (`PC13`) to start swing-up or trigger emergency stop.
