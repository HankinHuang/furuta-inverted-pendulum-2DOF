# 滤波相位延迟、极限环振荡与抑制策略

本文档总结离散 LQR 控制引入低通滤波后出现极限环振荡的机理，以及项目中验证的三种改进方案：延迟建模、增益调度 Soft-LQR 与基于机电模型的离散状态观测器。

---

## 1. 问题机理：滤波导致的相位延迟

一阶低通滤波器虽然有效抑制了编码器差分尖峰，但必然引入相位延迟（Phase Lag）。在 `Discrete_of_init_LQR.m` 中，在采样周期 $T_s = 0.001\text{ s}$ 下，角度滤波系数 $\alpha_{\text{angle}}$ 与输出滤波系数 $\alpha_{\text{output}}$ 共同引入的近似等效时间延迟为：

$$
\tau_{\text{delay}} \approx \frac{\alpha_{\text{angle}} T_s}{1 - \alpha_{\text{angle}}} + \frac{\alpha_{\text{output}} T_s}{1 - \alpha_{\text{output}}}
$$

由于倒立摆系统本身存在右半平面不稳定极点，过大的相位延迟会削减闭环相位裕度；叠加电机静摩擦死区与齿轮间隙后，实物平台在直立平衡点附近会出现持续的**极限环振荡（Limit Cycle）**（参见 `experiments/figures/Phase Portrait of Limit Cycle *.png` 与 `experiments/figures/Time-domain Delay Verification.png`）。

---

## 2. 改进策略一：控制输出滤波与增益调度 Soft-LQR

已部署于 `firmware/swingup_lqr/` 与 `firmware/swingup_soft_lqr/`：

1. **输出端指数平滑滤波**：对计算得到的电压/PWM 指令做一阶低通滤波（$u_k = \alpha u_{k-1} + (1-\alpha)u_{\text{raw},k}$），在不直接延迟原始角度反馈的前提下抑制高频执行器抖振。
2. **增益调度 Soft-LQR**（见 `firmware/swingup_soft_lqr/Core/Src/main.c`）：
   - **内圈**（$|\theta_1| < 0.070\text{ rad}$）：采用更柔和的反馈增益 `K_inner`，减弱直立位置附近由单步量化跳变与死区穿越引发的过激响应。
   - **中圈 / 外圈**（超过 `THRESHOLD_INNER_RAD` / `THRESHOLD_OUTER_RAD`）：切换至更强的 `K_mid` / `K_outer` 增益，确保较大偏角下的快速纠偏能力。

---

## 3. 改进策略二：基于观测器的 LQG 控制（Kalman / Luenberger）

已在 `simulation/lqr_kalman_filter/`（`init_LQR_Kalman.m` 与 `Model_LQR_Real_Kalman.slx`）中完成仿真验证：

### 3.1 建模关键约束

设计离散观测器矩阵 $(A_d, B_d)$ 时，**必须**基于包含电机反电动势阻尼（$c_\omega$）和电压-力矩系数（$c_V$）的机电联合矩阵 $(A_{\text{voltage}}, B_{\text{voltage}})$ 进行离散化，而不能仅使用纯机械 $(A, B)$ 矩阵；否则观测器预测的运动趋势会与电压驱动的实物系统失配。

### 3.2 嵌入式 C 语言离散更新参考

在 `FurutaController_UpdateMeasuredState` 中可按如下结构实现观测器更新：

```c
// 测量残差: y_err = y_meas - C_d * x_hat，其中 C_d 测量 [arm_angle, pend_angle]
float y_err[2] = {
    y_meas[0] - x_hat[0],
    y_meas[1] - x_hat[2]
};

// 状态预测与修正: x_hat_next = Ad * x_hat + Bd * u_prev + L * y_err
float x_hat_next[4] = {0.0f};
for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
        x_hat_next[i] += Ad[i][j] * x_hat[j];
    }
    x_hat_next[i] += Bd[i] * u_prev;
    x_hat_next[i] += L_obs[i][0] * y_err[0] + L_obs[i][1] * y_err[1];
}
```

其中 `u_prev` 为上一 $1\text{ ms}$ 控制周期实际施加给电机的电压值。
