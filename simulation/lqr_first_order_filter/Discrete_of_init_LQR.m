%% init_LQR.m - 初始化参数脚本
clear; 

%% 1. 物理参数定义 (Physical Parameters)
L_arm       = 0.1475;       % Arm length 旋臂长度 (m)
L_1         = 0.14;         % Pendulum length 摆杆长度 (m)
lcom_1      = 0.051;        % Pendulum center of mass distance 摆杆质心距离 (m)
m_1         = 0.054;        % Pendulum mass 摆杆质量 (kg)
g           = 9.8;          % Acceleration due to gravity 重力加速度
arm_torque  = 0;            % Initial torque 初始扭矩
omega_c     = 60;           % 滤波器截止频率
alpha_angle = 0.9;
alpha_output = 0.71;
DelayTime = alpha_angle*0.001/(1-alpha_angle) + alpha_output*0.001/(1-alpha_output); 

% --- Moment of inertia calculation 转动惯量计算 ---
% 1. Moment of inertia of the rod about its center of mass
% 1. 摆杆绕质心转动惯量
I_1 = 1.1e-4; 

% 2. 旋臂转动惯量 Moment of inertia of the rotating arm
I_arm = 7.39e-4; 

% 打印出来看看
fprintf('计算得到的 I_1 = %e kg*m^2\n', I_1);
fprintf('计算得到的 I_arm = %e kg*m^2\n', I_arm);

% 将物理参数打包，供 A_B_matrices 函数使用
% 顺序必须严格遵守：[L_arm, I_arm, lcom_1, I_1, m_1, g]
phys_param = [L_arm, I_arm, lcom_1, I_1, m_1, g];

%% 2. 摩擦参数 (Friction Parameters)
% 初始仿真可以设小一点，实测后填入真实值
f_v_arm = 0.002; f_c_arm = 0.008; k_fc_arm = 1;
f_v_1   = 0.0001; f_c_1   = 0.00056; k_fc_1   = 1;

% 打包摩擦参数
fric_arm = [f_v_arm, f_c_arm, k_fc_arm];
fric_1   = [f_v_1, f_c_1, k_fc_1];

%% 3. 电机参数
% --- 1. 定义电机参数 ---
V_nom = 12;           % 电机额定电压
N_gear = 9.267;       % 减速比
I_stall = 1.8;        % 堵转电流
I_0 = 0.07;           % 空载电流
RPM_out_noload = 620; % 空载转速
RPM_noload = RPM_out_noload * N_gear; % 空载转速

R_motor = V_nom / I_stall;        % 电机内阻 (欧姆)
w_noload = RPM_noload * 2 * pi / 60; % 空载角速度

Kt = V_nom / w_noload;          % 扭矩常数 (N.m/A)
Ke = (V_nom - I_0 * R_motor) / w_noload; % 反电动势常数 (V/(rad/s))

% --- 2. 计算转换系数 ---
% 电压 -> 扭矩的系数
c_V = (N_gear * Kt) / R_motor;  
% 速度 -> 阻尼扭矩的系数 (反电动势阻尼)
c_omega = (N_gear^2 * Kt * Ke) / R_motor; 

%% 3. 计算 LQR 增益 K (Calculate Gain)
% 调用你之前生成的函数计算线性化矩阵 A 和 B (纯机械连续模型)
[A, B] = furuta_single_A_B_matrices(0, phys_param, fric_arm, fric_1);

% 假设你的状态向量 x =[臂角; 臂速; 摆角; 摆速]
% 也就是说，A 矩阵的第 2 列对应的是"臂速"
A_new = A;

% 将反电动势阻尼加到大臂的动力学方程中
% B 矩阵是 4x1 的向量，代表扭矩对各个状态的导数影响
A_new(:, 2) = A(:, 2) - B * c_omega; 

% 将输入从"扭矩"变为"电压"，得到机电联合连续模型 (对应论文中的 A_voltage, B_voltage)
B_new = B * c_V;

%% 4. 计算离散 LQR (机电联合模型 - 适配 STM32)
% --- 1. 连续模型离散化 ---
T_s = 0.001; % 定义系统采样周期 1ms (根据你的 STM32 实际控制周期设定)

% 构建连续状态空间系统对象
sys_c = ss(A_new, B_new, eye(4), zeros(4,1));

% 使用零阶保持器 ('zoh') 将连续系统离散化
% 这一步将 MCU 的数字采样延迟和 PWM 保持效应在数学上精确建模
sys_d = c2d(sys_c, T_s, 'zoh');
G = sys_d.A; % 离散系统矩阵
H = sys_d.B; % 离散输入矩阵

% --- 2. LQR 调参区域 ---
% Q矩阵：状态权重 diag([旋臂角, 旋臂速, 摆杆角, 摆杆速])
% 想要摆杆更直？加大第3个参数(11)。想要旋臂不动？加大第1个参数(4)。
% R越小，电机越猛(Rigid)；R越大，电机越柔(Compliant)
Q = diag([4, 0.1, 11, 0.1]);
R = 0.7;

% --- 3. 求解离散代数黎卡提方程 (DARE) ---
% 使用 dlqr 替代 lqr，获取离散反馈增益
K_voltage = dlqr(G, H, Q, R);

% --- 4. 打印 K 值确认 ---
disp('================ 控制器增益计算结果 ================');
disp('1. 连续域基础 LQR Gain K (未加电机模型，仅供对比):');
disp(lqr(A, B, Q, R));

disp('2. 连续域机电联合 LQR Gain K (原算法，存在理论错位):');
disp(lqr(A_new, B_new, Q, R));

disp(['3. 离散域机电联合 DLQR Gain K (Ts = ', num2str(T_s*1000), 'ms, 最终部署至STM32):']);
disp(K_voltage);
disp('====================================================');

% --- 5. 转换到实际 C 代码使用的 K_set (考虑减速比状态映射) ---
% 理论模型算出的 K_voltage_discrete 是基于大臂角度的
% 如果 C 代码中直接使用的是电机轴角度 (未除以减速比)
% 则需要将前两项除以减速比 N_gear，实现状态映射的解耦

%% 4. 仿真初始条件 (Initial Conditions)
% 这是一个 4x1 的向量：[theta_arm; d_theta_arm; theta_pend; d_theta_pend]
% 设为 [0; 0; 0.1; 0] 表示摆杆初始偏离直立位置约 5.7 度，测试 LQR 能否救回来
init_S = [0; 0; 0.26; 0];

disp('参数初始化完成，可以运行 Simulink 了。');

%% 5. 频域分析：11.45ms 延迟对不同 R 值的影响 (Bode Plot)
disp('================ 正在绘制 Bode 图 ================');

% 1. 定义两个对比的 R 值
R = 20; % 理论最优但实机发散的 R
R_large = 0.7;    % 妥协后实机稳定的 R

% 2. 计算对应的离散 LQR 增益 K
K = dlqr(G, H, Q, R);
K_large = dlqr(G, H, Q, R_large);

% 3. 构建离散受控对象 P(z)
% sys_d 是之前 c2d 生成的离散状态空间模型
sys_P = ss(G, H, eye(4), zeros(4,1), T_s);

% 4. 引入 11.45ms 的纯延迟
% 采样周期 T_s = 1ms，所以 11.45ms 约等于 11 个采样周期的延迟 z^(-11)
delay_samples = round(DelayTime / T_s); 
sys_delay = tf(1,[1 zeros(1, delay_samples)], T_s); 

% 5. 计算断开控制输入处的开环环路增益 L(z) = K * P(z) * z^(-d)
% 注意：这是一个单输入单输出 (SISO) 的传递函数，非常适合画 Bode 图
L = K * sys_P * sys_delay;
L_large = K_large * sys_P * sys_delay;

% 6. 计算裕度
[Gm_s, Pm_s, Wcg_s, Wcp_s] = margin(L);[Gm_l, Pm_l, Wcg_l, Wcp_l] = margin(L_large);

% 7. 绘制 Bode 图
figure('Name', 'Bode Plot: Impact of Delay on LQR', 'Color', 'w');

% 画 R=0.007 的图
subplot(1,1,1);
margin(L);
title(sprintf('Aggressive LQR (R = %g)\nPhase Margin = %.2f deg', R, Pm_s), 'Color', 'r');
grid on;

% % 画 R=20 的图
% subplot(1,2,2);
% margin(L_large);
% title(sprintf('Conservative LQR (R = %g)\nPhase Margin = %.2f deg', R_large, Pm_l), 'Color', 'b');
% grid on;

disp('Bode 图绘制完成！请查看 Phase Margin (Pm)。');
disp(['R=', num2str(R), ' 时的相位裕度: ', num2str(Pm_s), ' 度 (若为负则不稳定)']);
disp(['R=', num2str(R_large), ' 时的相位裕度: ', num2str(Pm_l), ' 度 (若为正则稳定)']);
