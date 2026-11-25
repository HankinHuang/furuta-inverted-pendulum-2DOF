# Phase Lag, Limit-Cycle Oscillations, and Mitigation Strategies

This note summarizes the root cause of limit-cycle oscillations observed when applying low-pass filtering in discrete-time LQR control, together with the three engineering mitigations explored in this project: cascaded loop-delay accounting, gain-scheduled Soft-LQR, and model-based state observation.

---

## 1. Root Cause: Filter-Induced Phase Lag

First-order low-pass filtering smooths encoder quantization spikes at the cost of phase lag. In `Discrete_of_init_LQR.m`, the combined effective time delay from angle filtering ($\alpha_{\text{angle}}$) and control-output filtering ($\alpha_{\text{output}}$) at $T_s = 0.001\text{ s}$ is approximated as:

$$
\tau_{\text{delay}} \approx \frac{\alpha_{\text{angle}} T_s}{1 - \alpha_{\text{angle}}} + \frac{\alpha_{\text{output}} T_s}{1 - \alpha_{\text{output}}}
$$

For an inverted pendulum with an unstable right-half-plane pole, excessive phase lag reduces the closed-loop phase margin. Combined with motor dead-zone and gear backlash, this manifests on hardware as sustained **limit-cycle oscillations** around the upright equilibrium (see `experiments/figures/Phase Portrait of Limit Cycle *.png` and `experiments/figures/Time-domain Delay Verification.png`).

---

## 2. Mitigation 1: Control-Output Low-Pass Filtering & Gain-Scheduled Soft-LQR

Deployed in `firmware/swingup_lqr/` and `firmware/swingup_soft_lqr/`:

1. **Control-Output Smoothing**: Applying an exponential low-pass filter on the voltage/PWM command ($u_k = \alpha u_{k-1} + (1-\alpha)u_{\text{raw},k}$) prevents high-frequency motor chatter without overly delaying raw pendulum angle measurement.
2. **Gain-Scheduled Soft-LQR** (`firmware/swingup_soft_lqr/Core/Src/main.c`):
   - **Inner zone** ($|\theta_1| < 0.070\text{ rad}$): Uses softer gains (`K_inner`) to avoid over-reacting to single-tick quantization noise and dead-zone crossings near upright.
   - **Middle / Outer zones** (`K_mid`, `K_outer` beyond `THRESHOLD_INNER_RAD` / `THRESHOLD_OUTER_RAD`): Restores higher corrective authority when larger angular excursions occur.

---

## 3. Mitigation 2: Observer-Based LQG Control (Kalman / Luenberger)

Evaluated in `simulation/lqr_kalman_filter/` (`init_LQR_Kalman.m` and `Model_LQR_Real_Kalman.slx`):

### 3.1 Critical Modeling Requirement

When designing the discrete observer matrices $(A_d, B_d)$ from continuous-time dynamics, the observer **must** use the electromechanical matrices $(A_{\text{voltage}}, B_{\text{voltage}})$ that include motor back-EMF damping ($c_\omega$) and voltage-to-torque gain ($c_V$), rather than the pure mechanical $(A, B)$ matrices. Otherwise, the observer's velocity predictions diverge from the voltage-driven hardware dynamics.

### 3.2 Discrete Observer Update Structure (C Reference)

For embedded C deployment inside `FurutaController_UpdateMeasuredState`:

```c
// Residual: y_err = y_meas - C_d * x_hat, where C_d measures [arm_angle, pend_angle]
float y_err[2] = {
    y_meas[0] - x_hat[0],
    y_meas[1] - x_hat[2]
};

// State prediction + correction: x_hat_next = Ad * x_hat + Bd * u_prev + L * y_err
float x_hat_next[4] = {0.0f};
for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
        x_hat_next[i] += Ad[i][j] * x_hat[j];
    }
    x_hat_next[i] += Bd[i] * u_prev;
    x_hat_next[i] += L_obs[i][0] * y_err[0] + L_obs[i][1] * y_err[1];
}
```

Here `u_prev` is the motor voltage applied during the preceding $1\text{ ms}$ control interval.
