# Furuta 倒立摆物理建模与 LQR 控制推导

本文档使用拉格朗日力学推导 2-DOF Furuta（旋转）倒立摆的非线性动力学方程，将其整理为标准机器人动力学矩阵形式，并在直立平衡点附近进行线性化以求解 LQR 反馈增益矩阵。

---

## 1. 坐标与物理参数定义

- $\theta_{\text{arm}}$：旋臂角度（$\text{rad}$）
- $\theta_{1}$（或 $\theta_{\text{pend}}$）：摆杆角度（$\text{rad}$），其中 $\theta_1 = 0$ 为直立平衡点，$\theta_1 = \pi$ 为自然下垂点
- $L_{\text{arm}}$：旋臂有效长度（$\text{m}$）
- $L_{\text{com}}$：摆杆转轴到摆杆总质心的距离（$\text{m}$）
- $m_{\text{pend}}$：摆杆总质量（$\text{kg}$）
- $I_{\text{arm}}$：旋臂绕电机主轴的转动惯量（$\text{kg}\cdot\text{m}^2$）
- $I_{\text{pend}}$：摆杆绕自身质心的转动惯量（$\text{kg}\cdot\text{m}^2$）
- $g$：重力加速度（$\text{m/s}^2$）
- $f_{v,\text{arm}},\, f_{v,1}$：旋臂与摆杆关节的粘滞阻尼系数（$\text{N}\cdot\text{m}\cdot\text{s/rad}$）
- $f_{c,\text{arm}},\, f_{c,1}$：旋臂与摆杆关节的库伦摩擦力矩（$\text{N}\cdot\text{m}$）
- $k_{f,\text{arm}},\, k_{f,1}$：库伦摩擦连续光滑近似的双曲正切系数（$\text{s/rad}$）

---

## 2. 运动学建模

### 2.1 旋臂末端坐标

$$
\begin{aligned}
x_{\text{arm}} &= L_{\text{arm}} \cos(\theta_{\text{arm}}) \\
y_{\text{arm}} &= L_{\text{arm}} \sin(\theta_{\text{arm}}) \\
z_{\text{arm}} &= 0
\end{aligned}
$$

### 2.2 摆杆质心坐标（直立为零点）

$$
\begin{aligned}
x_{1,\text{com}} &= L_{\text{arm}} \cos(\theta_{\text{arm}}) - L_{\text{com}} \sin(\theta_1) \sin(\theta_{\text{arm}}) \\
y_{1,\text{com}} &= L_{\text{arm}} \sin(\theta_{\text{arm}}) + L_{\text{com}} \sin(\theta_1) \cos(\theta_{\text{arm}}) \\
z_{1,\text{com}} &= L_{\text{com}} \cos(\theta_1)
\end{aligned}
$$

*（注：$x_{1,\text{com}}, y_{1,\text{com}}$ 的交叉项正负号取决于 $\theta_{\text{arm}}$ 与 $\theta_1$ 的正方向定义；在直立为零点约定下，高度 $z_{1,\text{com}} = L_{\text{com}}\cos(\theta_1)$ 在 $\theta_1 = 0$ 时取最大值。）*

### 2.3 摆杆质心速度

对位置坐标求时间导数，得质心速度分量：

$$
\begin{aligned}
\dot{x}_{1,\text{com}} &= -L_{\text{arm}} \sin(\theta_{\text{arm}})\dot{\theta}_{\text{arm}} - L_{\text{com}}\left[\cos(\theta_1)\dot{\theta}_1\sin(\theta_{\text{arm}}) + \sin(\theta_1)\cos(\theta_{\text{arm}})\dot{\theta}_{\text{arm}}\right] \\
\dot{y}_{1,\text{com}} &= L_{\text{arm}} \cos(\theta_{\text{arm}})\dot{\theta}_{\text{arm}} + L_{\text{com}}\left[\cos(\theta_1)\dot{\theta}_1\cos(\theta_{\text{arm}}) - \sin(\theta_1)\sin(\theta_{\text{arm}})\dot{\theta}_{\text{arm}}\right] \\
\dot{z}_{1,\text{com}} &= -L_{\text{com}}\sin(\theta_1)\dot{\theta}_1
\end{aligned}
$$

---

## 3. 能量与欧拉-拉格朗日方程

### 3.1 动能与势能

总动能 $T$ 由摆杆平移动能、摆杆旋转动能和旋臂旋转动能组成：

$$
T = \frac{1}{2} m_{\text{pend}}\left(\dot{x}_{1,\text{com}}^2 + \dot{y}_{1,\text{com}}^2 + \dot{z}_{1,\text{com}}^2\right) + \frac{1}{2} I_{\text{pend}}\dot{\theta}_1^2 + \frac{1}{2} I_{\text{arm}}\dot{\theta}_{\text{arm}}^2
$$

系统重力势能 $P$ 为：

$$
P = m_{\text{pend}} g L_{\text{com}} \cos(\theta_1)
$$

### 3.2 拉格朗日量与广义力

取拉格朗日量 $\mathcal{L} = T - P$，广义坐标 $q = [\theta_{\text{arm}},\, \theta_1]^T$：

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}}\right) - \frac{\partial \mathcal{L}}{\partial q} = \tau
$$

为避免 $\operatorname{sgn}(\dot{\theta})$ 在过零点处不连续导致数值求解器震荡或卡死，库伦摩擦采用 $\tanh(k_f \dot{\theta})$ 进行光滑近似：

- **主动旋臂关节**：
  $$\tau_1 = \tau_{\text{arm}} - f_{v,\text{arm}}\dot{\theta}_{\text{arm}} - f_{c,\text{arm}}\tanh(k_{f,\text{arm}}\dot{\theta}_{\text{arm}})$$
- **被动（欠驱动）摆杆关节**：
  $$\tau_2 = -f_{v,1}\dot{\theta}_1 - f_{c,1}\tanh(k_{f,1}\dot{\theta}_1)$$

---

## 4. 标准机器人动力学矩阵形式

系统动力学可整理为标准形式：

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau
$$

### 4.1 惯性矩阵 $M(q)$

$$
M(q) = \begin{bmatrix} M_{11} & M_{12} \\ M_{21} & M_{22} \end{bmatrix}
$$

- $M_{11} = I_{\text{arm}} + m_{\text{pend}} L_{\text{arm}}^2 + m_{\text{pend}} L_{\text{com}}^2 \sin^2(\theta_1)$
- $M_{22} = I_{\text{pend}} + m_{\text{pend}} L_{\text{com}}^2$
- $M_{12} = M_{21} = -m_{\text{pend}} L_{\text{arm}} L_{\text{com}} \cos(\theta_1)$ *（符号取决于坐标正方向定义）*

### 4.2 科里奥利力与离心力向量 $C(q,\dot{q})\dot{q}$

$$
C(q,\dot{q})\dot{q} = \begin{bmatrix} C_1 \\ C_2 \end{bmatrix} = \begin{bmatrix} m_{\text{pend}} L_{\text{com}}^2 \sin(2\theta_1)\dot{\theta}_1\dot{\theta}_{\text{arm}} + m_{\text{pend}} L_{\text{arm}} L_{\text{com}} \sin(\theta_1)\dot{\theta}_1^2 \\ -\frac{1}{2} m_{\text{pend}} L_{\text{com}}^2 \sin(2\theta_1)\dot{\theta}_{\text{arm}}^2 \end{bmatrix}
$$

### 4.3 重力向量 $G(q)$

$$
G(q) = \begin{bmatrix} 0 \\ -m_{\text{pend}} g L_{\text{com}} \sin(\theta_1) \end{bmatrix}
$$

---

## 5. 状态空间建模、机电耦合与 LQR 设计

### 5.1 非线性状态空间方程

对 $M(q)$ 求逆可得显式角加速度 $\ddot{q} = M(q)^{-1}(\tau - C(q,\dot{q})\dot{q} - G(q))$。定义状态向量 $x$ 与控制输入 $u = \tau_{\text{arm}}$：

$$
x = \begin{bmatrix} \theta_{\text{arm}} \\ \dot{\theta}_{\text{arm}} \\ \theta_1 \\ \dot{\theta}_1 \end{bmatrix}, \qquad
\dot{x} = f(x, u) = \begin{bmatrix} \dot{\theta}_{\text{arm}} \\ f_1(x, u) \\ \dot{\theta}_1 \\ f_2(x, u) \end{bmatrix}
$$

### 5.2 雅可比线性化与电机电压映射

在直立平衡点 $x_e = 0,\, u_e = 0$ 处求雅可比矩阵，得到纯机械连续模型：

$$
A = \left.\frac{\partial f}{\partial x}\right|_{x_e, u_e}, \qquad B = \left.\frac{\partial f}{\partial u}\right|_{x_e, u_e}
$$

为将控制输入从扭矩 $\tau_{\text{arm}}$ 映射为电机驱动电压 $V$ 并计入反电动势阻尼（对应 `Discrete_of_init_LQR.m`）：

$$
c_V = \frac{N_{\text{gear}} K_t}{R_{\text{motor}}}, \qquad c_\omega = \frac{N_{\text{gear}}^2 K_t K_e}{R_{\text{motor}}}
$$

$$
A_{\text{voltage}}(:, 2) = A(:, 2) - B \cdot c_\omega, \qquad B_{\text{voltage}} = B \cdot c_V
$$

### 5.3 LQR 最优控制求解

在连续或零阶保持离散化的状态空间模型下，LQR 最小化无限时间二次型代价函数：

$$
J = \int_0^\infty \left(x^T Q x + u^T R u\right) dt
$$

求解代数黎卡提方程（CARE/DARE）得到最优反馈增益矩阵 $K$，闭环控制律为 $u = -Kx$。
