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

## Experimental Results Preview

| Limit Cycle Comparison (LQR vs. Soft-LQR) | Time-Domain Delay Verification |
| :---: | :---: |
| ![Phase Portrait SoftLQR](experiments/figures/Phase%20Portrait%20of%20Limit%20Cycle%20SoftLQR.png) | ![Delay Verification](experiments/figures/Time-domain%20Delay%20Verification.png) |

- Demonstration video: [`media/demo_video.mp4`](media/demo_video.mp4)

https://github.com/user-attachments/assets/3cab052c-f99d-4dfd-ab25-deb1932a8f66

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
  - Huaicheng Huang, Ruisen He, Esther Chiu, Yingjie Guan, Janet Ngai 
- **Supervisor**: Dr. Andrew Harrison
- **Base Dynamics Reference**: The initial Furuta pendulum Simulink state-derivative and animation framework in `simulation/pid_swingup/` was adapted from the open-source `furuta-inverted-pendulum` base dynamics template (see `simulation/pid_swingup/README_base_model.md`) and extended by our team for hardware parameters, swing-up, cascade PID, discrete LQR, and filtering.
