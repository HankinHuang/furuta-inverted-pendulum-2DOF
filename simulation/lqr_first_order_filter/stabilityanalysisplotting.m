clear; clc; close all;

% 替换为你的 LQR Simulink 模型名称 (不要带 .slx)
mdl = 'Model_LQR_Real_First_order';   
load_system(mdl);

% ===== 干扰持续时间扫描列表 (单位: ms) =====
% 对应你实机测试的 20ms, 25ms, 30ms, 35ms, 40ms
dist_duration_ms_list = [30, 40, 50, 55];   

% 干扰开始时间
t_dist = 0.5;
assignin('base', 't_dist', t_dist); % 发送到工作区供 Simulink 的 Step1 使用

% 存储结构体
distSweep = struct();

for i = 1:length(dist_duration_ms_list)
    dist_duration_ms = dist_duration_ms_list(i);
    
    % 将 ms 转换为 秒，并发送到工作区供 Simulink 的 Step2 使用
    dist_duration_s = dist_duration_ms / 1000; 
    assignin('base', 'dist_duration', dist_duration_s); 

    % 运行仿真
    simOut = sim(mdl);

    % 读取变量
    theta_pend = simOut.get('theta_pend');
    theta_arm  = simOut.get('theta_arm');

    % 读取电机电压 (建议你在 Simulink 中把最终输出到电机的电压用 To Workspace 记录下来)
    % 如果你记录的还是 torque，把下面的 motor_voltage 改回 motor_torque_limited 即可
    try
        motor_voltage = simOut.get('motor_voltage');
    catch
        try
            motor_voltage = simOut.get('ut.motor_voltage');
        catch
            % 兼容旧代码，如果没改名的话
            motor_voltage = simOut.get('motor_torque_limited'); 
        end
    end

    distSweep(i).duration_ms = dist_duration_ms;

    distSweep(i).t_pend = theta_pend.Time(:);
    distSweep(i).y_pend = squeeze(theta_pend.Data);

    distSweep(i).t_arm  = theta_arm.Time(:);
    distSweep(i).y_arm  = squeeze(theta_arm.Data);

    distSweep(i).t_u    = motor_voltage.Time(:);
    distSweep(i).y_u    = squeeze(motor_voltage.Data);
end

%% ===== Figure 1: Pendulum angle disturbance sweep =====
figure('Color','w','Position',[100 100 1100 750]);
hold on; grid on; box on;

for i = 1:length(distSweep)
    plot(distSweep(i).t_pend, distSweep(i).y_pend, 'LineWidth', 2.2, ...
        'DisplayName', sprintf('100%% PWM for %d ms', distSweep(i).duration_ms));
end

h1 = xline(t_dist, '--', 'LineWidth', 1.2, ...
    'FontSize', 16, 'LabelVerticalAlignment', 'middle');
h1.HandleVisibility = 'off';

xlabel('Time (s)', 'FontSize', 22, 'FontWeight', 'bold');
ylabel('Pendulum angle (rad)', 'FontSize', 22, 'FontWeight', 'bold');
title('Pendulum Angle Under 100% PWM Disturbance', 'FontSize', 24, 'FontWeight', 'bold');

lgd = legend('Location','northwest');
lgd.FontSize = 16;

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.3;
xlim([0 6]);

exportgraphics(gcf, 'disturbance_sweep_pendulum_ms.emf', 'ContentType', 'vector');

%% ===== Figure 2: Arm angle disturbance sweep =====
figure('Color','w','Position',[120 120 1100 750]);
hold on; grid on; box on;

for i = 1:length(distSweep)
    plot(distSweep(i).t_arm, distSweep(i).y_arm, 'LineWidth', 2.2, ...
        'DisplayName', sprintf('100%% PWM for %d ms', distSweep(i).duration_ms));
end

h2 = xline(t_dist, '--', 'LineWidth', 1.2, ...
    'FontSize', 16, 'LabelVerticalAlignment', 'middle');
h2.HandleVisibility = 'off';

xlabel('Time (s)', 'FontSize', 22, 'FontWeight', 'bold');
ylabel('Arm angle (rad)', 'FontSize', 22, 'FontWeight', 'bold');
title('Arm Angle Under 100% PWM Disturbance', 'FontSize', 24, 'FontWeight', 'bold');

lgd = legend('Location','northwest');
lgd.FontSize = 16;

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.3;
xlim([0 6]);

exportgraphics(gcf, 'disturbance_sweep_arm_ms.emf', 'ContentType', 'vector');

%% ===== Figure 3: Motor Control Effort (Voltage/Torque) =====
figure('Color','w','Position',[140 140 1100 750]);
hold on; grid on; box on;

for i = 1:length(distSweep)
    plot(distSweep(i).t_u, distSweep(i).y_u, 'LineWidth', 2.2, ...
        'DisplayName', sprintf('100%% PWM for %d ms', distSweep(i).duration_ms));
end

h3 = xline(t_dist, '--', 'LineWidth', 1.2, ...
    'FontSize', 16, 'LabelVerticalAlignment', 'middle');
h3.HandleVisibility = 'off';

xlabel('Time (s)', 'FontSize', 22, 'FontWeight', 'bold');
ylabel('Control Effort', 'FontSize', 22, 'FontWeight', 'bold'); % 如果你输出的是电压，改为 Voltage (V)
title('Control Effort Under 100% PWM Disturbance', 'FontSize', 24, 'FontWeight', 'bold');

lgd = legend('Location','northwest');
lgd.FontSize = 16;

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.3;
xlim([0 6]);

exportgraphics(gcf, 'disturbance_sweep_effort_ms.emf', 'ContentType', 'vector');