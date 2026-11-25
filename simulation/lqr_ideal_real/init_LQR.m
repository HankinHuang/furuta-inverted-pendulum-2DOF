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
alpha       = 0.8;

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
f_v_arm = 0.002; f_c_arm = 0.002; k_fc_arm = 1;
f_v_1   = 0.0001; f_c_1   = 0.0001; k_fc_1   = 1;

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
% 调用你之前生成的函数计算线性化矩阵 A 和 B
[A, B] = furuta_single_A_B_matrices(0, phys_param, fric_arm, fric_1);

% 假设你的状态向量 x =[臂角; 臂速; 摆角; 摆速]
% 也就是说，A 矩阵的第 2 列对应的是"臂速"
A_new = A;

% 将反电动势阻尼加到大臂的动力学方程中
% B 矩阵是 4x1 的向量，代表扭矩对各个状态的导数影响
A_new(:, 2) = A(:, 2) - B * c_omega; 

% 将输入从"扭矩"变为"电压"
B_new = B * c_V;

% --- LQR 调参区域 ---
% Q矩阵：状态权重 diag([旋臂角, 旋臂速, 摆杆角, 摆杆速])
% 想要摆杆更直？加大第3个参数(100)。想要旋臂不动？加大第1个参数。
% R越小，电机越猛(Rigid)；R越大，电机越柔(Compliant)

% --- 4. 计算 LQR (机电联合模型) ---
% 此时的 Q 和 R 矩阵，R 代表的是对"电压"的惩罚 "消耗R(V)电压的代价，和摆杆偏离1弧度的代价是相等的"
% 参数
Q = diag([pi, (pi/128), (pi/6), (pi/36)]);
R = 6;
K = lqr(A, B, Q, R);
K_voltage = lqr(A_new, B_new, Q, R);

% --- 5. 转换到实际 C 代码使用的 K_set (考虑减速比状态映射) ---
% 理论模型算出的 K_voltage 是基于大臂角度的
% 如果 C 代码中直接使用的是电机轴角度 (未除以减速比)
% 则需要将前两项除以减速比 N_gear，实现状态映射的解耦

% 打印 K 值确认
disp('LQR Gain K 计算完成:');
disp(K);
disp('考虑电机模型后的 K 矩阵 (输入为电压V):');
disp(K_voltage);

%% 4. 仿真初始条件 (Initial Conditions)
% 这是一个 4x1 的向量：[theta_arm; d_theta_arm; theta_pend; d_theta_pend]
% 设为 [0; 0; 0.1; 0] 表示摆杆初始偏离直立位置约 5.7 度，测试 LQR 能否救回来
init_S = [0; 0; 0.26; 0];

disp('参数初始化完成，可以运行 Simulink 了。');