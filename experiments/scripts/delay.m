% =========================================================================
% 倒立摆系统实机数据验证脚本 (手动界定时间版)
% =========================================================================
clear; clc; close all;

%% 1. 读取并解析数据
filename = 'R20.csv';
data = readmatrix(filename);

% 提取需要的列 (MATLAB为1-based index)
time_raw                = data(:, 1);  % 原始时间戳
arm_angle_filtered      = data(:, 3);  % Filtered Arm Angle
pend_angle_filtered     = data(:, 5);  % Pendulum Angle (已滤波)
pend_vel_filtered       = data(:, 6);  % Pendulum Velocity (已滤波)
arm_angle_raw           = data(:, 8)+ pi;  % Raw Arm Angle (未滤波)
pend_angle_raw          = data(:, 9)+ pi;  % Raw Arm Angle (未滤波)
motor_voltage_unfiltered= data(:, 2);  % Motor Voltage unfiltered
motor_voltage_filtered  = data(:, 7);  % Motor Voltage filtered

%% 2. 手动界定分析的时间窗口 (请在此处修改)
% =========================================================
% 请输入你想截取的原始数据时间段 (单位：秒)
manual_start_time = 12;  % 设定的开始时间
manual_end_time   = 12.5;  % 设定的结束时间
% =========================================================

% 2. 图一纵坐标范围设置 (Arm Angle)
arm_y_limit = 0.2;        % 设定 Arm Angle 的显示范围为 [-0.1, 0.1]
% =========================================================

% 根据设定的时间寻找对应的数组索引
start_idx = find(time_raw >= manual_start_time, 1, 'first');
end_idx   = find(time_raw <= manual_end_time, 1, 'last');

% 防错保护：确保索引有效
if isempty(start_idx), start_idx = 1; end
if isempty(end_idx), end_idx = length(time_raw); end
if start_idx >= end_idx
    error('结束时间必须大于开始时间！请检查 manual_start_time 和 manual_end_time 的设置。');
end

% 截取指定时间段的数据
time_slice              = time_raw(start_idx:end_idx);
arm_raw_slice           = arm_angle_raw(start_idx:end_idx);
arm_filt_slice          = arm_angle_filtered(start_idx:end_idx);

pend_angle_raw          = pend_angle_raw(start_idx:end_idx);
pend_ang_slice          = pend_angle_filtered(start_idx:end_idx);
pend_vel_slice          = pend_vel_filtered(start_idx:end_idx);

voltage_unfilter_slice  = motor_voltage_unfiltered(start_idx:end_idx);
voltage_slice           = motor_voltage_filtered(start_idx:end_idx);
% 【重要】将截取后的时间戳首位归零，让 X 轴从 0 开始
time_slice = time_slice - time_slice(1);

%% 3. 绘制图一：实机波形局部放大对比图 (延迟验证)
figure('Name', 'Time-domain Delay Verification', 'Color', 'w', 'Position', [100, 100, 800, 450]);

% 左轴：角度对比 (Raw vs Filtered)
yyaxis left
plot(time_slice, pend_angle_raw, '--', 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5, 'DisplayName', 'Unfiltered Pend Angle');
hold on;
plot(time_slice, pend_ang_slice, '-', 'Color', [0.3010 0.7450 0.9330], 'LineWidth', 1.5, 'DisplayName', 'Filtered Pend Angle');
ylabel('Pend Angle (rad)');
set(gca, 'YColor', 'k');

% 【新增功能】：手动界定左侧(Arm Angle)的Y轴范围
ylim([-arm_y_limit, arm_y_limit]);

% 右轴：电压输出
yyaxis right
plot(time_slice, voltage_slice, '-', 'Color', [1 0 0], 'LineWidth', 1.5, 'DisplayName', 'Voltage Filtered');
hold on;
plot(time_slice, voltage_unfilter_slice, '--', 'Color', [1 0 0], 'LineWidth', 1.5, 'DisplayName', 'Voltage Unfiltered');
ylabel('Motor Voltage (V)');
set(gca, 'YColor', 'r');

% 图表修饰
xlabel('Time (s) - Normalized to 0');
title('Time-domain Delay Verification');
grid on;
legend('Location', 'northeast');

% --- 局部放大窗口设置 ---
% 注意：这里的 zoom_start 是相对于"归零后"的时间。
% 例如，如果你截取的片段是 0.6s 到 4.0s，zoom_start = 0.4 表示看原数据的 1.0s 处。
zoom_start = 0.4;               % 放大区间的起始时间 (归零后的时间)
zoom_end   = zoom_start + 0.15; % 放大 0.15 秒的窗口
xlim([zoom_start, zoom_end]); 

% 开启数据光标，方便测算差值
datacursormode on; 


%% 4. 绘制图二：相平面轨迹图 (极限环)
figure('Name', 'Phase Portrait of Limit Cycle', 'Color', 'w', 'Position', [950, 100, 600, 500]);

% 画出手动截取时间段内的所有背景轨迹（浅灰色）
plot(pend_ang_slice, pend_vel_slice, '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 1, 'DisplayName', 'Full Slice Trajectory');
hold on;

% 提取该段最后 1/3 的数据着重画极限环（黑色粗线），避开刚开始截取时可能的暂态
limit_cycle_start_idx = floor(length(pend_ang_slice) * 2 / 3);

plot(pend_ang_slice(limit_cycle_start_idx:end), pend_vel_slice(limit_cycle_start_idx:end), ...
    'k-', 'LineWidth', 1.5, 'DisplayName', 'Steady-state Limit Cycle');

% 标出原点 (0,0)
plot(0, 0, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'Ideal Origin (0,0)');

% 图表修饰
xlabel('Pendulum Angle (rad)');
ylabel('Pendulum Velocity (rad/s)');
title(sprintf('Phase Portrait (Original Time Range: %.1fs - %.1fs)', manual_start_time, manual_end_time));
grid on;
legend('Location', 'best');

% 聚焦到中心震荡区域（视数据范围可以微调此处）
xlim([-0.05, 0.05]); 
ylim([-1, 1]);