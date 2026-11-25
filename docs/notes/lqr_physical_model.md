# Furuta Pendulum Physical Modeling and LQR Formulation

This document derives the nonlinear equations of motion of the 2-DOF Furuta (rotary) inverted pendulum using Lagrangian mechanics, transforms them into standard manipulator matrix form, and linearizes the dynamics about the upright equilibrium for LQR controller synthesis.

---

## 1. Coordinate & Parameter Definitions

- $\theta_{\text{arm}}$: Rotary arm angle ($\text{rad}$)
- $\theta_{1}$ (or $\theta_{\text{pend}}$): Pendulum angle ($\text{rad}$), where $\theta_1 = 0$ corresponds to the upright equilibrium and $\theta_1 = \pi$ corresponds to the downward hanging position
- $L_{\text{arm}}$: Rotary arm length ($\text{m}$)
- $L_{\text{com}}$: Distance from the pendulum pivot to the pendulum center of mass ($\text{m}$)
- $m_{\text{pend}}$: Total pendulum mass ($\text{kg}$)
- $I_{\text{arm}}$: Moment of inertia of the rotary arm about the motor axis ($\text{kg}\cdot\text{m}^2$)
- $I_{\text{pend}}$: Moment of inertia of the pendulum about its center of mass ($\text{kg}\cdot\text{m}^2$)
- $g$: Gravitational acceleration ($\text{m/s}^2$)
- $f_{v,\text{arm}},\, f_{v,1}$: Viscous damping coefficients of the arm and pendulum joints ($\text{N}\cdot\text{m}\cdot\text{s/rad}$)
- $f_{c,\text{arm}},\, f_{c,1}$: Coulomb friction torques of the arm and pendulum joints ($\text{N}\cdot\text{m}$)
- $k_{f,\text{arm}},\, k_{f,1}$: Hyperbolic tangent smoothing gains for continuous Coulomb friction approximation ($\text{s/rad}$)

---

## 2. Kinematics

### 2.1 Arm Tip Position

$$
\begin{aligned}
x_{\text{arm}} &= L_{\text{arm}} \cos(\theta_{\text{arm}}) \\
y_{\text{arm}} &= L_{\text{arm}} \sin(\theta_{\text{arm}}) \\
z_{\text{arm}} &= 0
\end{aligned}
$$

### 2.2 Pendulum Center-of-Mass Position (Measured Relative to Upright)

$$
\begin{aligned}
x_{1,\text{com}} &= L_{\text{arm}} \cos(\theta_{\text{arm}}) - L_{\text{com}} \sin(\theta_1) \sin(\theta_{\text{arm}}) \\
y_{1,\text{com}} &= L_{\text{arm}} \sin(\theta_{\text{arm}}) + L_{\text{com}} \sin(\theta_1) \cos(\theta_{\text{arm}}) \\
z_{1,\text{com}} &= L_{\text{com}} \cos(\theta_1)
\end{aligned}
$$

*(Note: Sign conventions in $x_{1,\text{com}}, y_{1,\text{com}}$ depend on the positive rotation direction chosen for $\theta_{\text{arm}}$ and $\theta_1$; in the upright convention $z_{1,\text{com}} = L_{\text{com}}\cos(\theta_1)$ peaks at $\theta_1 = 0$.)*

### 2.3 Pendulum Center-of-Mass Velocity

Differentiating the position coordinates with respect to time yields $\dot{x}_{1,\text{com}},\, \dot{y}_{1,\text{com}},\, \dot{z}_{1,\text{com}}$:

$$
\begin{aligned}
\dot{x}_{1,\text{com}} &= -L_{\text{arm}} \sin(\theta_{\text{arm}})\dot{\theta}_{\text{arm}} - L_{\text{com}}\left[\cos(\theta_1)\dot{\theta}_1\sin(\theta_{\text{arm}}) + \sin(\theta_1)\cos(\theta_{\text{arm}})\dot{\theta}_{\text{arm}}\right] \\
\dot{y}_{1,\text{com}} &= L_{\text{arm}} \cos(\theta_{\text{arm}})\dot{\theta}_{\text{arm}} + L_{\text{com}}\left[\cos(\theta_1)\dot{\theta}_1\cos(\theta_{\text{arm}}) - \sin(\theta_1)\sin(\theta_{\text{arm}})\dot{\theta}_{\text{arm}}\right] \\
\dot{z}_{1,\text{com}} &= -L_{\text{com}}\sin(\theta_1)\dot{\theta}_1
\end{aligned}
$$

---

## 3. Energy & Euler-Lagrange Equations

### 3.1 Kinetic and Potential Energy

The total kinetic energy $T$ combines pendulum translation, pendulum rotation, and arm rotation:

$$
T = \frac{1}{2} m_{\text{pend}}\left(\dot{x}_{1,\text{com}}^2 + \dot{y}_{1,\text{com}}^2 + \dot{z}_{1,\text{com}}^2\right) + \frac{1}{2} I_{\text{pend}}\dot{\theta}_1^2 + \frac{1}{2} I_{\text{arm}}\dot{\theta}_{\text{arm}}^2
$$

The gravitational potential energy $P$ is:

$$
P = m_{\text{pend}} g L_{\text{com}} \cos(\theta_1)
$$

### 3.2 Lagrangian & Generalized Forces

With Lagrangian $\mathcal{L} = T - P$ and generalized coordinates $q = [\theta_{\text{arm}},\, \theta_1]^T$:

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}}\right) - \frac{\partial \mathcal{L}}{\partial q} = \tau
$$

To avoid numerical discontinuities in ODE solvers at $\dot{\theta}=0$, Coulomb friction is modeled using $\tanh(k_f \dot{\theta})$ instead of $\operatorname{sgn}(\dot{\theta})$:

- **Active rotary arm joint:**
  $$\tau_1 = \tau_{\text{arm}} - f_{v,\text{arm}}\dot{\theta}_{\text{arm}} - f_{c,\text{arm}}\tanh(k_{f,\text{arm}}\dot{\theta}_{\text{arm}})$$
- **Passive (underactuated) pendulum joint:**
  $$\tau_2 = -f_{v,1}\dot{\theta}_1 - f_{c,1}\tanh(k_{f,1}\dot{\theta}_1)$$

---

## 4. Standard Manipulator Matrix Form

The coupled nonlinear dynamics can be written in standard form:

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau
$$

### 4.1 Inertia Matrix $M(q)$

$$
M(q) = \begin{bmatrix} M_{11} & M_{12} \\ M_{21} & M_{22} \end{bmatrix}
$$

- $M_{11} = I_{\text{arm}} + m_{\text{pend}} L_{\text{arm}}^2 + m_{\text{pend}} L_{\text{com}}^2 \sin^2(\theta_1)$
- $M_{22} = I_{\text{pend}} + m_{\text{pend}} L_{\text{com}}^2$
- $M_{12} = M_{21} = -m_{\text{pend}} L_{\text{arm}} L_{\text{com}} \cos(\theta_1)$ *(sign follows the axis orientation convention)*

### 4.2 Coriolis & Centrifugal Vector $C(q,\dot{q})\dot{q}$

$$
C(q,\dot{q})\dot{q} = \begin{bmatrix} C_1 \\ C_2 \end{bmatrix} = \begin{bmatrix} m_{\text{pend}} L_{\text{com}}^2 \sin(2\theta_1)\dot{\theta}_1\dot{\theta}_{\text{arm}} + m_{\text{pend}} L_{\text{arm}} L_{\text{com}} \sin(\theta_1)\dot{\theta}_1^2 \\ -\frac{1}{2} m_{\text{pend}} L_{\text{com}}^2 \sin(2\theta_1)\dot{\theta}_{\text{arm}}^2 \end{bmatrix}
$$

### 4.3 Gravity Vector $G(q)$

$$
G(q) = \begin{bmatrix} 0 \\ -m_{\text{pend}} g L_{\text{com}} \sin(\theta_1) \end{bmatrix}
$$

---

## 5. State-Space Formulation, Electromechanical Coupling & LQR

### 5.1 Nonlinear State-Space Representation

Inverting $M(q)$ gives explicit angular accelerations $\ddot{q} = M(q)^{-1}(\tau - C(q,\dot{q})\dot{q} - G(q))$. Defining the state vector $x$ and control input $u = \tau_{\text{arm}}$:

$$
x = \begin{bmatrix} \theta_{\text{arm}} \\ \dot{\theta}_{\text{arm}} \\ \theta_1 \\ \dot{\theta}_1 \end{bmatrix}, \qquad
\dot{x} = f(x, u) = \begin{bmatrix} \dot{\theta}_{\text{arm}} \\ f_1(x, u) \\ \dot{\theta}_1 \\ f_2(x, u) \end{bmatrix}
$$

### 5.2 Jacobian Linearization & Motor Voltage Mapping

Linearizing $f(x,u)$ about the upright equilibrium $x_e = 0,\, u_e = 0$ yields the mechanical continuous-time matrices:

$$
A = \left.\frac{\partial f}{\partial x}\right|_{x_e, u_e}, \qquad B = \left.\frac{\partial f}{\partial u}\right|_{x_e, u_e}
$$

To convert the control input from torque $\tau_{\text{arm}}$ to motor voltage $V$ and include motor back-EMF damping (`Discrete_of_init_LQR.m`):

$$
c_V = \frac{N_{\text{gear}} K_t}{R_{\text{motor}}}, \qquad c_\omega = \frac{N_{\text{gear}}^2 K_t K_e}{R_{\text{motor}}}
$$

$$
A_{\text{voltage}}(:, 2) = A(:, 2) - B \cdot c_\omega, \qquad B_{\text{voltage}} = B \cdot c_V
$$

### 5.3 LQR Optimization

With continuous (or ZOH-discretized) state-space matrices, LQR minimizes the infinite-horizon quadratic cost:

$$
J = \int_0^\infty \left(x^T Q x + u^T R u\right) dt
$$

Solving the Continuous/Discrete Algebraic Riccati Equation yields the optimal state-feedback gain $K$, giving control law $u = -Kx$.
