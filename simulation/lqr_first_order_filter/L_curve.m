clear; clc; close all;

% 1. 运行你的初始化脚本，获取最准确的物理参数和离散矩阵 G, H
% 确保 Discrete_of_init_LQR.m 在当前文件夹中
Discrete_of_init_LQR; 

% ================= 新增修复部分 =================
% 为了防止 Simulink 中的 Step 模块报错，定义干扰时间在仿真时间之外 (例如 999 秒)
% 这样在测试稳态 L-Curve 时，就不会触发 100% PWM 干扰
t_dist = 999; 
dist_duration = 0;
assignin('base', 't_dist', t_dist);
assignin('base', 'dist_duration', dist_duration);
% ================================================

% 此时工作区已经有了 G, H, Q 以及 Simulink 需要的所有物理参数
% 我们只需要重新定义 R 的扫描范围
R_test = logspace(0, 1.7, 10); % 生成 20 个对数分布的点，从 1 到 50

% 存储结果
rms_theta = zeros(length(R_test), 1);
rms_u = zeros(length(R_test), 1);
K3_values = zeros(length(R_test), 1);

mdl = 'Model_LQR_Real_First_order'; % 你的高保真仿真模型
load_system(mdl);

disp('开始扫描 R 值并运行仿真，请稍候...');

for i = 1:length(R_test)
    R_current = R_test(i);
    
    % 计算当前 R 下的离散 LQR 增益 (注意这里用的是 G 和 H)
    [K, ~, ~] = dlqr(G, H, Q, R_current);
    
    assignin('base', 'K_voltage', K); % 发送到工作区给 Simulink
    K3_values(i) = K(3); % 记录 K3 的值
    
    % 运行仿真
    simOut = sim(mdl);
    
    % 提取数据 (加入 try-catch 兼容不同的 Simulink 输出格式)
    try
        t = simOut.get('theta_pend').Time;
        theta_data = squeeze(simOut.get('theta_pend').Data);
        
        % 尝试获取控制电压/扭矩数据
        try
            u_data = squeeze(simOut.get('motor_torque_limited').Data);
        catch
            u_data = squeeze(simOut.get('ut.motor_torque_limited').Data);
        end
    catch
        t = simOut.get('out.theta_pend').Time;
        theta_data = squeeze(simOut.get('out.theta_pend').Data);
        u_data = squeeze(simOut.get('out.motor_torque_limited').Data);
    end
    
    % 假设稳态从 0.5 秒开始计算 (避开初始 0.01 rad 的下落瞬态)
    idx = find(t > 0.2); 
    
    % 计算 RMS
    rms_theta(i) = rms(theta_data(idx));
    rms_u(i) = rms(u_data(idx));
    
    fprintf('进度: %d/%d | R = %.2f | K3 = %.2f\n', i, length(R_test), R_current, K(3));
end

% 3. 绘制 L-Curve (帕累托前沿)
figure('Color','w','Position',[100 100 800 600]);
plot(rms_theta, rms_u, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
grid on;
xlabel('RMS of Pendulum Angle (rad)[Accuracy]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('RMS of Control Voltage (V) [Smoothness]', 'FontSize', 14, 'FontWeight', 'bold');
title('L-Curve for Soft Mode R Tuning (Pareto Front)', 'FontSize', 16, 'FontWeight', 'bold');

% 在图上标出 R 的值 (每隔一个点标一次，防止文字重叠)
for i = 1:2:length(R_test)
    text(rms_theta(i), rms_u(i), sprintf('  R=%.1f', R_test(i)), 'FontSize', 10);
end

disp('扫描完成！');