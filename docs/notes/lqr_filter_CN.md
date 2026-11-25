# LQR 控制中的速度估计与滤波方法

由于硬件编码器仅提供角度位置信号，如何在 $1\text{ kHz}$（$T_s = 0.001\text{ s}$）控制周期下从量化角度中提取低噪声、低相移的角速度信号，是实现 LQR 稳定控制的关键。

---

## 1. 直接差分引起的量化噪声

在采样周期 $T_s = 0.001\text{ s}$ 下，直接后向差分为：

$$
v_k = \frac{\theta_k - \theta_{k-1}}{T_s}
$$

当摆杆编码器跳变 1 个最小计数（$\Delta\theta_1 = 2\pi / 4096 \approx 0.00153\text{ rad}$）时，瞬时差分速度会跳升至 $\approx 1.53\text{ rad/s}$，而在未跳变周期内又回落为 $0\text{ rad/s}$。该脉冲尖峰经由 LQR 速度增益 $K_4$ 放大后，会引起电机剧烈抖振与发热。

---

## 2. 方案一：一阶低通滤波器（LPF）

带低通滤波的微分器计算开销极低，适合嵌入式实时部署。

### 2.1 连续与离散形式

- **连续域滤波微分器**：
  $$G(s) = \frac{\omega_c s}{s + \omega_c}$$
  其中 $\omega_c = 2\pi f_c$ 为截止角频率（通常取 $f_c \in [10, 20]\text{ Hz}$，即 $\omega_c \approx 60\text{–}125\text{ rad/s}$）。

- **离散域指数平滑滤波**（已部署于 `firmware/swingup_lqr/` 与 `firmware/swingup_soft_lqr/`）：
  $$v_k = \alpha v_{k-1} + (1 - \alpha)\frac{\theta_k - \theta_{k-1}}{T_s}, \qquad \alpha \in (0, 1)$$

### 2.2 工程权衡

- $\alpha$ 越大（$\omega_c$ 越低），速度曲线越平滑、电机高频抖振越小，但引入的**相位延迟**越严重（近似等效延迟 $\tau_{\text{delay}} \approx \frac{\alpha T_s}{1 - \alpha}$）。
- $\alpha$ 越小（$\omega_c$ 越高），相位延迟越小，但高频量化噪声会直接穿透至控制输出。
- 过大的相位延迟会消耗不稳定开环极点的相位裕度，从而诱发低频极限环振荡。

---

## 3. 方案二：基于物理模型的状态观测器（Luenberger / 稳态卡尔曼滤波器）

为在抑制噪声的同时尽量消除相位延迟，可利用机电联合离散模型进行速度预测（见 `simulation/lqr_kalman_filter/`）。

### 3.1 数学形式

基于零阶保持离散化后的机电联合矩阵 $(A_d, B_d)$ 与角度测量矩阵 $C_d$（仅测旋臂角与摆杆角）：

$$
C_d = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 \end{bmatrix}
$$

离散状态观测器在每个采样周期按如下递推更新状态估计 $\hat{x}_k$：

$$
\hat{x}_{k+1} = A_d \hat{x}_k + B_d u_k + L\left(y_k - C_d \hat{x}_k\right)
$$

其中 $y_k = [\theta_{\text{arm},k},\, \theta_{1,k}]^T$，观测器增益 $L \in \mathbb{R}^{4\times 2}$ 可通过 LQR 对偶形式（稳态卡尔曼滤波器）求解：

```matlab
Q_obs = diag([1e-4, 1e-2, 1e-4, 1e-2]); % 过程噪声协方差（模型可信度）
R_obs = diag([1e-2, 1e-2]);             % 测量噪声协方差（传感器噪声水平）
L = dlqr(Ad', Cd', Q_obs, R_obs)';
```

### 3.2 Simulink 实现方式

在 `Model_LQR_Real_Kalman.slx` 中，通过 `MATLAB Function` 模块接收量化角度向量 `y_measured` 与上一拍控制电压 `u`，并将 `Ad`、`Bd`、`Cd`、`L` 配置为工作区参数：

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

## 4. 方案对比总结

| 对比维度 | 一阶低通滤波器（LPF） | 稳态卡尔曼 / Luenberger 观测器 |
| :--- | :--- | :--- |
| **计算复杂度** | 极低（标量递推） | 中等（$4\times 4$ 与 $4\times 2$ 矩阵向量乘加） |
| **模型依赖性** | 不依赖动力学模型 | 依赖包含电机特性的准确 $(A_d, B_d)$ 模型 |
| **相位延迟** | $\omega_c$ 较低时延迟明显 | 极小（模型前馈预测速度） |
| **工程状态** | 已在 STM32 固件部署（`swingup_lqr`, `swingup_soft_lqr`） | 已在 Simulink 完成仿真验证（`lqr_kalman_filter/`） |
