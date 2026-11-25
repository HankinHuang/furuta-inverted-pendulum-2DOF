# System Physical Parameters

Measured physical and electromechanical parameters used across MATLAB/Simulink modeling (`simulation/`) and STM32 firmware (`firmware/`), compiled from `Parameters.docx` and `Discrete_of_init_LQR.m`.

## 1. Rotary Arm Parameters

| Parameter | Symbol | Value | Unit | Notes |
| :--- | :--- | :--- | :--- | :--- |
| Arm rod mass | $m_{\text{rod}}$ | 0.030 | kg | Uniform rod |
| Housing mass | $m_{\text{housing}}$ | 0.047 | kg | Pendulum encoder/potentiometer housing |
| Total arm mass | $m_{\text{arm}}$ | 0.077 | kg | $m_{\text{rod}} + m_{\text{housing}}$ |
| Arm length (pivot to pendulum axis) | $L_{\text{arm}}$ | 0.1475 | m | Effective kinematic arm length (`init_LQR.m`) |
| Full rod length | $l_{\text{rod}}$ | 0.22 | m | Physical rod length |
| Rod CoM offset from arm pivot | $l_{\text{pivot,com}}$ | 0.020 | m | — |
| Housing offset from arm pivot | $l_{\text{housing,pivot}}$ | 0.114 | m | Nominal 0.11 m (`I_housing` uses 0.114 m) |
| Arm moment of inertia about pivot | $I_{\text{arm}}$ | $7.39 \times 10^{-4}$ | $\text{kg}\cdot\text{m}^2$ | $I_{\text{rod,pivot}} + I_{\text{housing,pivot}}$ |

## 2. Pendulum Parameters

| Parameter | Symbol | Value | Unit | Notes |
| :--- | :--- | :--- | :--- | :--- |
| Swing rod mass | $m_{\text{swing}}$ | 0.042 | kg | — |
| Flange connection mass | $m_{\text{flange}}$ | 0.012 | kg | — |
| Total pendulum mass | $m_1$ | 0.054 | kg | $m_{\text{swing}} + m_{\text{flange}}$ |
| Pendulum total length | $L_1$ | 0.14 | m | — |
| Pendulum overall CoM from pivot | $l_{\text{com},1}$ | 0.051 | m | Weighted CoM of swing arm + flange |
| Pendulum inertia about overall CoM | $I_1$ | $1.1 \times 10^{-4}$ | $\text{kg}\cdot\text{m}^2$ | $I_{\text{swing,com}} + I_{\text{flange,com}}$ |

## 3. Identified Electromechanical & Friction Parameters

| Parameter | Symbol | Value | Unit |
| :--- | :--- | :--- | :--- |
| Nominal motor voltage | $V_{\text{nom}}$ | 12.0 | V |
| Gear ratio | $N_{\text{gear}}$ | 9.267 | — |
| Stall current | $I_{\text{stall}}$ | 1.8 | A |
| No-load current | $I_0$ | 0.07 | A |
| Output no-load speed | $\text{RPM}_{\text{out,noload}}$ | 620 | rpm |
| Arm viscous / Coulomb friction | $f_{v,\text{arm}}, f_{c,\text{arm}}$ | 0.002, 0.008 | $\text{Nms/rad}$, $\text{Nm}$ |
| Pendulum viscous / Coulomb friction | $f_{v,1}, f_{c,1}$ | 0.0001, 0.00056 | $\text{Nms/rad}$, $\text{Nm}$ |
