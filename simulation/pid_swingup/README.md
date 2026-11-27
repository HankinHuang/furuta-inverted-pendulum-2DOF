# Furuta Dynamics, Swing-Up & Cascade PID Simulink Models

MATLAB and Simulink models covering the 2-DOF Furuta nonlinear plant dynamics, autonomous swing-up verification, and dual-loop cascade PID simulation.

## Key Files

- **Base Plant Dynamics & Animation Framework**:
  - `furuta_single_inverted_pendulum_live.mlx` / `furuta_single_inverted_pendulum_nonlive_copy.m`: Symbolic Euler-Lagrange derivation and state-space linearization.
  - `functions/furuta_single_state_derivative.m` & `functions/furuta_single_A_B_matrices.m`: Nonlinear state-derivative and linearized $(A, B)$ matrix functions.
  - See [README_base_model.md](README_base_model.md) for the original base dynamics template documentation and parameter conventions.
- **Swing-Up Control**:
  - `Swing_UP_esther.slx`: Simulink verification of the feedback-linearized Lyapunov energy swing-up controller with velocity filtering, continuous $\tanh$ approximation, and arm-centering PD controller.
- **Cascade PID Simulation & Hardware Deployment Models**:
  - `PID_correection_final_simulation.slx` & `PID_correection_final_simulation_quantizer_simulation.slx`: Dual-loop cascade PID tuning and quantization simulations.
  - `PID_correection_ideal_parametertest.slx` & `PID_correection_hardware.slx`: Parameter sweep and hardware-matched Simulink models.

## Running in MATLAB

Add `functions/` to the MATLAB path (or run via the Simulink model `InitFcn`), open the target `.slx` model, and use fixed-step `ode4` (`Ts = 0.001 s`).
