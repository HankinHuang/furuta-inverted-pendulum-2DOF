% 古田单倒立摆非实时副本
% Resources used: Dynamic Modeling and Simulation of a Rotational Inverted 所用资源：旋转倒立摆的动态建模与仿真
% Pendulum (J L Duart et al 2017 J. Phys.: Conf. Ser. 792 012081) 钟摆（J L Duart等人，2017年，《物理学报：会议系列》792卷012081号）
 
% L_arm is the length of the arm. / L_arm 是手臂的长度
% I_arm is the moment of inertia of the arm. / I_arm 是手臂的转动惯量

% lcom_1 is the distance between the arm-connected end of the pendulum and the pendulum's center of mass. 
% lcom_1是摆臂连接端与摆锤质心之间的距离
% I_1 is the moment of inertia of the pendulum along its length (negligible moment of inertia assumed along the width and height of the pendulum)
% I_1 是摆锤沿其长度方向的转动惯量（假设摆锤沿宽度和高度方向的转动惯量可忽略不计）
% m_1 is the mass of the pendulum / m_1 是摆锤的质量

% Friction terms include viscous and coulomb. / 摩擦项包括粘性和库仑摩擦。
% Viscous friction = viscous_friction_coefficient*velocity / 粘性摩擦 = 粘性摩擦系数 * 速度
% Coulomb friction = coulomb_friction_torque * sign(velocity) <- this sign / 库仑摩擦力 = 库仑摩擦扭矩 * sign(速度) <- 这个符号
% function introduces discontinuity at v=0 and so is modelled by tanh(k*v) / 该函数在v=0处引入不连续性，因此通过tanh(k*v)进行建模
% instead, where k is the smoothing gain / 相反，其中k是平滑增益
% f_v_arm is the viscous damping coefficient of the arm / f_v_arm是手臂的粘性阻尼系数
% f_c_arm is the coulomb friction torque (F_n*coefficient) of the arm / f_c_arm是手臂的库仑摩擦力矩（F_n*系数）
% f_v_1 is the viscous damping coefficient of the pendulum / f_v_1是摆的粘性阻尼系数
% f_c_1 is the coulomb friction torque (F_n*coefficient) of the pendulum / f_c_1是摆锤的库仑摩擦力矩（F_n*系数）
% k_fc_arm is the coulomb friction smoothing gain for the arm (used in tanh to produce a continuous sign(v_theta_arm) effect)
% k_fc_arm 是手臂的库仑摩擦平滑增益（用于 tanh 函数以产生连续的 sign(v_theta_arm) 效果）
% k_fc_1 is the coulomb friction smoothing gain for the pendulum (used in tanh to produce a continuous sign(v_theta_1) effect)
% k_fc_1 是摆杆的库仑摩擦平滑增益（用于 tanh 函数以产生连续的 sign(v_theta_1) 效果）
syms L_arm I_arm lcom_1 I_1 m_1 g
syms f_v_arm f_c_arm k_fc_arm
syms f_v_1 f_c_1 k_fc_1
syms arm_torque
syms theta_arm(t) theta_1(t)

%%
% X and Y and Z positions of the end of arm / 机械臂末端的 X、Y 和 Z 位置
x_arm = L_arm*cos(theta_arm);
y_arm = L_arm*sin(theta_arm);
z_arm = 0;

% X and Y and Z positions of the pendulum COM / 摆锤质心的X、Y和Z位置
x_1_com = x_arm - lcom_1*sin(theta_1)*sin(theta_arm);
y_1_com = y_arm + lcom_1*sin(theta_1)*cos(theta_arm);
z_1_com = z_arm + lcom_1*cos(theta_1);

% X and Y and Z velocities of the pendulum COM / 摆锤质心的X、Y和Z速度
v_x1_com = diff(x_1_com);
v_y1_com = diff(y_1_com);
v_z1_com = diff(z_1_com);

% Angular velocities from theta_arm and theta_1 / 来自theta_arm和theta_1的角速度
v_theta_arm = diff(theta_arm);
v_theta_1 = diff(theta_1);
v_funcs = [v_theta_arm, v_theta_1];
acc_funcs = [diff(v_theta_arm), diff(v_theta_1)];

% Translational KE of the pendulum 摆锤的平移动能
T_1_translational = (1/2)*m_1*(v_x1_com^2 + v_y1_com^2 + v_z1_com^2);

% Rotational KE of the pendulum 摆锤的旋转动能
T_1_rotational = (1/2)*I_1*v_theta_1^2;

% Rotational KE of the arm 手臂的旋转动能
T_arm_rotational = (1/2)*I_arm*v_theta_arm^2;

% Total KE 总动能
T_total = T_1_translational + T_1_rotational + T_arm_rotational;

% PE of the pendulum (setting the zero PE height to be the arm height) 摆的势能（将零势能高度设为臂高）
P_1 = m_1*g*z_1_com;

% Total PE (The arm moves in the horizontal plane and has zero PE) 总势能（手臂在水平面内运动，势能为零）
P_total = P_1;

% Lagrangian 拉格朗日
L = T_total - P_total;

% EOM for the degrees of freedoms (theta_arm and theta_1) 自由度（theta_arm和theta_1）的欧拉-拉格朗日方程
% Extra forces such as friction/disturbances to be added here 在此处添加额外的力，如摩擦力/干扰
eq_arm_LHS = diff(diff(L, v_theta_arm)) - diff(L, theta_arm);
eq_arm_RHS = arm_torque - f_v_arm*v_theta_arm - f_c_arm*tanh(k_fc_arm*v_theta_arm);
eq_1_LHS = diff(diff(L, v_theta_1)) - diff(L, theta_1);
eq_1_RHS = - f_v_1*v_theta_1 - f_c_1*tanh(k_fc_1*v_theta_1);
eq_arm = simplify(eq_arm_LHS == eq_arm_RHS);
eq_1 = simplify(eq_1_LHS == eq_1_RHS);

M = simplify(jacobian([eq_arm_LHS, eq_1_LHS] , acc_funcs));
C_x = simplify([eq_arm_LHS; eq_1_LHS] - M*acc_funcs.' - G.');
C = simplify(jacobian(C_x, v_funcs)); % TODO: C has an extra 2 due to differentiation of theta_dot^2 right now 待办：由于当前对θ_dot²进行了微分处理，C中多了一个2
G = simplify(subs([eq_arm_LHS, eq_1_LHS], [v_funcs, acc_funcs], [0,0,0,0]));

% Define the accelerations as syms variables 将加速度定义为符号变量
syms a_theta_arm a_theta_1

% Find the diff(v_theta_arm), diff(v_theta_1) terms in eq_arm, eq_1, and replace with a_theta_arm, a_theta_1 correspondingly 
% 在方程eq_arm和eq_1中找到diff(v_theta_arm)和diff(v_theta_1)项，并分别替换为a_theta_arm和a_theta_1
acc_vars = [a_theta_arm, a_theta_1];
eqs_sub = subs([eq_arm, eq_1], acc_funcs, acc_vars);

% Solve for the acceleration terms 求解加速度项
sol_acc = solve(eqs_sub, acc_vars);
a_theta_arm = simplify(sol_acc.a_theta_arm);
a_theta_1 = simplify(sol_acc.a_theta_1);

% S (state) is defined as [theta_arm; v_theta_arm; theta_1; v_theta_1] 
% 状态 S 定义为 [theta_arm; v_theta_arm; theta_1; v_theta_1] 
% Define state derivative 定义状态导数
dSdt = [v_theta_arm; a_theta_arm; v_theta_1; a_theta_1];

% Create symbolic vector to store the state 创建符号向量以存储状态
S_var = sym('S_var', [4 1])

% Replace all S terms (syms funcs) in dSdt with S_var (syms variables) 将dSdt中的所有S项（符号函数）替换为S_var（符号变量）
% *Important*: Replace the higher order terms first: e.g. if theta_arm is replaced first with S_var(1), diff(theta_arm) becomes diff(S_var(1)) such that later MATLAB cannot find 'diff(theta_arm)' to replace with S_var(2)
% *重要提示*：首先替换高阶项：例如，如果先使用S_var(1)替换theta_arm，那么diff(theta_arm)就会变成diff(S_var(1))，这样MATLAB之后就无法找到'diff(theta_arm)'并用S_var(2)来替换它了
dSdt = subs(dSdt, [diff(theta_arm), diff(theta_1), theta_arm, theta_1], [S_var(2), S_var(4), S_var(1), S_var(3)]);

% Obtain A and B matrices in state-space equation, linearised about the equilibrium point (arm position at 0, no arm velocity, upright position, no pendulum velocity)
% 获取状态空间方程中的A和B矩阵，在平衡点（手臂位置为0，无手臂速度，直立位置，无摆速度）处进行线性化
equilibrium = [0; 0; 0 ;0];
A_mat = simplify(jacobian(dSdt, S_var));
A_linear = simplify(subs(A_mat, S_var, equilibrium));
B_mat = simplify(jacobian(dSdt, arm_torque));
B_linear = simplify(subs(B_mat, S_var, equilibrium));

% Define parameters used in the functions below 定义以下函数中使用的参数
phys_param = [L_arm, I_arm, lcom_1, I_1, m_1, g];
fric_arm = [f_v_arm, f_c_arm, k_fc_arm];
fric_1 = [f_v_1, f_c_1, k_fc_1];

% Create a file containing the functions to be used in simulation 创建一个包含模拟中要使用的函数的文件
% This function outputs dSdt given inputs t, S_var, arm_torque, phys_param, fric_arm, fric_1 
% 该函数根据输入 t、S_var、arm_torque、phys_param、fric_arm、fric_1 输出 dSdt
matlabFunction(dSdt, 'File', 'functions/furuta_single_state_derivative', 'Vars', {t, S_var, arm_torque, phys_param, fric_arm, fric_1});
% This function outputs A_linear and B_linear given inputs t, phys_param, fric_arm, fric_1
% 此函数根据输入参数 t、phys_param、fric_arm 和 fric_1 输出 A_linear 和 B_linear。
matlabFunction(A_linear, B_linear, 'File', 'functions/furuta_single_A_B_matrices', 'Vars', {t, phys_param, fric_arm, fric_1});