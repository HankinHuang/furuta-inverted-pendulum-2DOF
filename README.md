# 2-DOF Furuta Rotary Inverted Pendulum

**Design, Build, and Test of a Non-Linear Dynamic System** — Electromechanical Group Project (Group G2025), School of Electrical, Electronic, and Mechanical Engineering, University of Bristol (2025–2026).

This repository contains the embedded STM32 firmware, MATLAB/Simulink models, mechanical CAD files, electrical documentation, experimental logs, and the final technical report for an autonomous swing-up and upright-balancing Furuta pendulum testbed.

---

## Key Highlights

- **Full Hardware Platform**: Custom-machined rotary arm & pendulum assembly, conductive slip ring for continuous 360° arm rotation, dual rotary encoders, DC motor drive, and STM32 Nucleo-F446RE embedded controller running at **1 kHz**.
- **Autonomous Swing-Up**: Energy-based & feedback-linearization swing-up controller (`furuta_swingup.c`) with smooth transition logic into the upright balancing region.
- **Two Stabilisation Strategies**:
  1. **Cascade PID Baseline** (`firmware/swingup_pid/`, `simulation/pid_swingup/`): Practical baseline controller with derivative filtering.
  2. **Discrete-Time LQR & Soft-LQR** (`firmware/swingup_lqr/`, `firmware/swingup_soft_lqr/`): Linearized electromechanical state-space model with zero-order hold (`c2d`, $T_s = 1\text{ ms}$), first-order low-pass filtering on control output, and gain-scheduled **Soft-LQR** (`K_inner` / `K_mid` / `K_outer`) to suppress limit-cycle oscillations caused by encoder quantization and loop delay.
- **Sim-to-Real Verification**: High-fidelity Simulink models incorporating motor back-EMF damping, viscous/Coulomb friction, quantization, and delay verification against physical rig CSV logs.

---

## Repository Structure

```text
├── README.md
├── LICENSE
├── docs/
│   ├── Final_Report_G2025.pdf          # Complete final project report (PDF)
│   ├── LQR_Presentation.pptx           # Control design presentation slides
│   ├── system_parameters.md            # Summary table of physical & motor parameters
│   └── notes/                          # Technical notes (EN + `_CN.md` Chinese companions)
├── firmware/                           # STM32CubeIDE C projects (STM32F446RE)
│   ├── swingup_lqr/                    # Swing-up + discrete LQR with 1st-order LPF
│   ├── swingup_soft_lqr/               # Swing-up + gain-scheduled Soft-LQR with 1st-order LPF
│   └── swingup_pid/                    # Swing-up + cascade PID controller
├── simulation/                         # MATLAB & Simulink models
│   ├── lqr_ideal_real/                 # Ideal & realistic Simulink LQR models + animations
│   ├── lqr_first_order_filter/         # Discrete LQR (`Discrete_of_init_LQR.m`) & filter models
│   ├── lqr_kalman_filter/              # LQR with Kalman state estimation (`Model_LQR_Real_Kalman.slx`)
│   └── pid_swingup/                    # Base Furuta dynamics & cascade PID / swing-up models
├── hardware/
│   ├── cad/                            # SolidWorks parts (`.SLDPRT`) and laser-cutting profiles (`.DXF`)
│   └── electrical/                     # BOM, pinout, and wiring notes (EN + `_CN.md` companions)
├── experiments/
│   ├── data/                           # Physical hardware telemetry logs (`.csv`)
│   ├── scripts/                        # MATLAB post-processing (`Graphs.m`, `Phase.m`, `delay.m`, etc.)
│   └── figures/                        # Bode plots, limit-cycle phase portraits, oscilloscope & disturbance sweeps
└── media/
    └── demo_video.mp4                  # Hardware demonstration clip
```

---

## Experimental Results & Controller Comparison

Full plots and derivations for all subsystems are documented in [`docs/Final_Report_G2025.pdf`](docs/Final_Report_G2025.pdf) (Chapter 6).

### 1. Swing-Up & Mode Switching (Sections 6.1–6.2)

- **Autonomous Swing-Up**: Using feedback linearization + Lyapunov energy control (`tanh` smoothing + first-order velocity filter, $\alpha = 0.9$), physical hardware achieved swing-up within **0.188 s** of the nominal simulation time with minimal chattering; inherent friction bounded arm sweep to **35.3°** even without the arm-centering PD term.
- **Switching Logic**: Across 20 hardware trials per threshold (with $|\dot{\theta}_p| < 5\text{ rad/s}$, $|\dot{\theta}_a| < 5\text{ rad/s}$, `10 ms` debounce), a pendulum-angle threshold of **0.25 rad** achieved **100% switching success** (vs. 85% at `0.15 rad`) without hitting post-switch actuator saturation seen at `0.35–0.45 rad`.

### 2. Balancing Performance: Cascade PID vs. Discrete LQR (Sections 6.3–6.5)

| Metric | Cascade PID (`firmware/swingup_pid/`) | Discrete LQR (`firmware/swingup_lqr/`, `R = 20`) |
| :--- | :--- | :--- |
| **Control Structure** | Dual-loop cascade PD/PID (`1 ms` inner / `7 ms` outer) | State-feedback DLQR (`Ts = 1 ms`) + cascaded LPF ($\alpha_{\text{angle}}=0.9$, $\alpha_{\text{output}}=0.71$) |
| **Simulation Settling** | `1.26 s` pendulum ($\pm 0.01\text{ rad}$), `1.29 s` arm ($\pm 0.02\text{ rad}$) | Stabilized for $R \ge 0.7$ (+50.7° phase margin); tolerates `50 ms` 12 V pulse |
| **Hardware Capture & Steady State** | Captures in `0.54 s`; `~7.5 Hz` residual limit cycle (`+0.109` to `-0.107 rad`, `0.216 rad` p-p); arm within $\pm 0.05\text{ rad}$ | Suppresses chattering (+48.2° PM, `18.4 rad/s` BW); smaller steady-state limit cycle (~$\pm 0.035\text{ rad}$ in Fig. 56) |
| **100% PWM Disturbance (`20 / 25 ms`)** | Recovers reliably | Recovers reliably |
| **100% PWM Disturbance (`30 ms`)** | Marginal recovery (~4/10 trials, multiple arm rotations) | Recovers reliably |
| **100% PWM Disturbance (`40 ms`)** | Loses balance (physical actuator limit) | Loses balance (physical actuator limit) |

### 3. Visual Previews & Demo Video

*(Note: Standalone figures and `.csv` logs in `experiments/` cover the LQR characterization suite; Swing-Up, Cascade PID time-domain/phase-portrait plots, and PID-vs-LQR disturbance comparison plots are presented in Sections 6.1–6.3 and 6.5 of [`docs/Final_Report_G2025.pdf`](docs/Final_Report_G2025.pdf).)*

| Limit Cycle Comparison (LQR vs. Soft-LQR) | Time-Domain Delay Verification |
| :---: | :---: |
| ![Phase Portrait SoftLQR](experiments/figures/Phase%20Portrait%20of%20Limit%20Cycle%20SoftLQR.png) | ![Delay Verification](experiments/figures/Time-domain%20Delay%20Verification.png) |

https://github.com/user-attachments/assets/3cab052c-f99d-4dfd-ab25-deb1932a8f66

- Source clip: [`media/demo_video.mp4`](media/demo_video.mp4)
- Complete theoretical derivation, hardware design, and experimental evaluation: [`docs/Final_Report_G2025.pdf`](docs/Final_Report_G2025.pdf)

---

## Quick Start

### 1. MATLAB / Simulink Simulation
1. Open MATLAB and navigate to the desired folder under `simulation/` (e.g., `simulation/lqr_first_order_filter/`).
2. Run the initialization script first to populate workspace parameters and controller gains:
   - **Discrete LQR**: run `Discrete_of_init_LQR.m` (or `init_LQR.m`), then open `Model_LQR_Real_First_order.slx`.
   - **Kalman Filter LQR**: run `init_LQR_Kalman.m`, then open `Model_LQR_Real_Kalman.slx`.
   - **PID & Base Dynamics**: see `simulation/pid_swingup/README_base_model.md` and `PID_correection_hardware.slx`.

### 2. STM32 Firmware (STM32CubeIDE)
1. Open **STM32CubeIDE** and import any of the three projects from `firmware/`:
   - `firmware/swingup_lqr/` — Swing-up + LQR with 1st-order low-pass filter
   - `firmware/swingup_soft_lqr/` — Swing-up + gain-scheduled Soft-LQR with 1st-order low-pass filter
   - `firmware/swingup_pid/` — Swing-up + cascade PID
2. Build (`Project -> Build Project`) and flash to **Nucleo-F446RE** via ST-LINK.
3. Optional UART telemetry is streamed over `USART2` every `50 ms`; press the onboard blue user button (`PC13`) to toggle/latch emergency stop during balancing.

### 3. Hardware & CAD
- Mechanical parts and DXF cutting files are located in `hardware/cad/`.
- Electrical component list and pinout/wiring notes are in `hardware/electrical/`.

---

## Team & Acknowledgments

- **Project Team (Group G2025, University of Bristol)**:
  - **Huaicheng Huang**: Electromechanical modeling, Simulink simulation, LQR/Soft-LQR & filtering implementation
  - **Ruisen He**: Simulink simulation, system integration, and testing of cascade PID control
  - **Esther Chiu**: Dynamic modeling, Simulink simulation, integration, and testing of swing-up control
  - **Yingjie Guan**: Mechanical design/manufacturing, system parameter identification, and disturbance testing
  - **Janet Ngai**: Electrical and embedded implementation, switching logic integration/testing, and PCB design
- **Supervisor**: Dr. Andrew Harrison
- **Base Dynamics Reference**: The initial Furuta pendulum Simulink state-derivative and animation framework in `simulation/pid_swingup/` was adapted from the open-source `furuta-inverted-pendulum` base dynamics template (see `simulation/pid_swingup/README_base_model.md`) and extended by our team for hardware parameters, swing-up, cascade PID, discrete LQR, and filtering.
