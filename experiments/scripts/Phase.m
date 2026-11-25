% 读取 CSV 文件并保留包含特定符号（如 "."）的原始列名
opts = detectImportOptions('R20.csv');
opts.VariableNamingRule = 'preserve';
data = readtable('R20.csv', opts);

% 提取时间和滤波后的状态量
time = data.('time');
pendulum_angle = data.('g_controller.measured.pendulum_angle_rad');
pendulum_velocity = data.('g_controller.measured.pendulum_velocity_rad_s');

% 计算目标时间范围
t0 = time(1);
t_start = t0 + 11.5;
t_end = t0 + 12.5;

% 逻辑索引提取特定时间段数据
idx = (time >= t_start) & (time <= t_end);
angle_window = pendulum_angle(idx);
velocity_window = pendulum_velocity(idx);

% 绘制相位图
figure;
plot(angle_window, velocity_window, 'b-', 'LineWidth', 1.5);
grid on;

% 图表格式化
xlabel('Filtered Pendulum Angle (rad)');
ylabel('Filtered Pendulum Angular Velocity (rad/s)');
title(sprintf('Filtered Pendulum Phase Portrait (t = %.3fs to %.3fs)', t_start, t_end));