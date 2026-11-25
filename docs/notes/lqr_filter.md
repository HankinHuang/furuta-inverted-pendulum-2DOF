# Velocity Estimation and Filtering for LQR Control

Because the hardware encoders measure angular position only, estimating angular velocity from quantized measurements at $1\text{ kHz}$ ($T_s = 0.001\text{ s}$) without introducing excessive noise or destabilizing phase lag is critical for LQR stability.

---

## 1. Finite-Difference Quantization Noise

Direct numerical differentiation over $T_s = 0.001\text{ s}$ computes:

$$
v_k = \frac{\theta_k - \theta_{k-1}}{T_s}
$$

When an encoder transitions by one count ($\Delta\theta_1 = 2\pi / 4096 \approx 0.00153\text{ rad}$), the instantaneous differentiated speed spikes to $\approx 1.53\text{ rad/s}$, followed by $0\text{ rad/s}$ in cycles with no count change. Multiplying these spikes by the LQR velocity gain $K_4$ causes severe actuator chatter and heating.

---

## 2. Approach A: First-Order Low-Pass Filter (LPF)

A filtered differentiator attenuates high-frequency quantization spikes with minimal computational cost.

### 2.1 Continuous & Discrete Formulations

- **Continuous-time filtered differentiator**:
  $$G(s) = \frac{\omega_c s}{s + \omega_c}$$
  where $\omega_c = 2\pi f_c$ is the cutoff frequency in $\text{rad/s}$ (typically $f_c \in [10, 20]\text{ Hz}$, i.e., $\omega_c \approx 60\text{–}125\text{ rad/s}$).

- **Discrete-time exponential smoothing** (implemented in `firmware/swingup_lqr/` and `firmware/swingup_soft_lqr/`):
  $$v_k = \alpha v_{k-1} + (1 - \alpha)\frac{\theta_k - \theta_{k-1}}{T_s}, \qquad \alpha \in (0, 1)$$

### 2.2 Engineering Trade-Off

- Larger $\alpha$ (lower $\omega_c$) produces smoother velocities and less motor jitter, but introduces **phase lag** (effective delay $\tau_{\text{delay}} \approx \frac{\alpha T_s}{1 - \alpha}$).
- Smaller $\alpha$ (higher $\omega_c$) reduces phase lag, but passes more quantization noise to the motor.
- Excessive phase lag erodes phase margin on the unstable inverted pendulum poles and induces low-frequency limit-cycle oscillations.

---

## 3. Approach B: Model-Based State Observer (Luenberger / Steady-State Kalman Filter)

To suppress quantization noise while minimizing phase lag, a discrete state observer predicts velocities from the electromechanical plant model (`simulation/lqr_kalman_filter/`).

### 3.1 Formulation

Using the ZOH-discretized electromechanical matrices $(A_d, B_d)$ and output measurement matrix $C_d$ (measuring $\theta_{\text{arm}}$ and $\theta_1$ only):

$$
C_d = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 \end{bmatrix}
$$

The observer updates the estimated state $\hat{x}_k$ at each sample:

$$
\hat{x}_{k+1} = A_d \hat{x}_k + B_d u_k + L\left(y_k - C_d \hat{x}_k\right)
$$

where $y_k = [\theta_{\text{arm},k},\, \theta_{1,k}]^T$ and $L \in \mathbb{R}^{4\times 2}$ is the observer gain computed via duality with LQR (steady-state Kalman filter):

```matlab
Q_obs = diag([1e-4, 1e-2, 1e-4, 1e-2]); % Process noise covariance (model confidence)
R_obs = diag([1e-2, 1e-2]);             % Measurement noise covariance (sensor noise)
L = dlqr(Ad', Cd', Q_obs, R_obs)';
```

### 3.2 Simulink Implementation

Inside `Model_LQR_Real_Kalman.slx`, the observer is implemented in a `MATLAB Function` block taking quantized angle vector `y_measured` and previous applied voltage `u`, with `Ad`, `Bd`, `Cd`, `L` configured as workspace parameters:

```matlab
function x_hat = state_observer(y_measured, u, Ad, Bd, Cd, L)
    persistent x_est;
    if isempty(x_est)
        x_est = zeros(4, 1);
    end
    x_hat = x_est;
    y_hat = Cd * x_est;
    x_est = Ad * x_est + Bd * u + L * (y_measured - y_hat);
end
```

---

## 4. Comparison Summary

| Criterion | First-Order LPF | Steady-State Kalman / Luenberger Observer |
| :--- | :--- | :--- |
| **Computational Cost** | Very low (scalar recurrence) | Moderate ($4\times 4$ and $4\times 2$ matrix-vector products) |
| **Model Dependency** | Model-free | Requires accurate $(A_d, B_d)$ including motor dynamics |
| **Phase Lag** | Noticeable at low $\omega_c$ | Minimal (model feedforward predicts velocity) |
| **Firmware Deployment** | Deployed on STM32 (`swingup_lqr`, `swingup_soft_lqr`) | Evaluated in Simulink (`lqr_kalman_filter/`) |
