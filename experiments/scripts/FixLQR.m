% 1. 读取CSV文件
filename = 'R20.csv';
data = readtable(filename);

% 2. 更改参数名
% 原始列名为: time, g_controller_measured_arm_angle_rad, g_controller_measured_arm_velocity_rad_s, 等
% 定义新的参数名，数量必须与原始列数一致（此处为5列）
newNames = {'Time', 'Voltage u', 'ArmAngle', 'ArmVelocity', 'PendulumAngle', 'PendulumVelocity', 'u filtered', 'raw_arm_angle', 'raw_pend_angle'};
data.Properties.VariableNames = {'Time', 'debug_unfiltered_u', 'ArmAngle', 'ArmVelocity', 'PendulumAngle', 'PendulumVelocity', 'raw_control', 'raw_arm_angle', 'raw_pend_angle'};

% ==================== 核心逻辑修改 ====================
% 3. 先将整列时间归零！
% 这样无论CSV里存的是 50s 还是 1442s 起步，都在这一步被强制变成了 0s 起步
data.Time = data.Time - data.Time(1);

% 4. 然后再进行筛选（现在可以直接写你想看的相对时间了，比如 9s 到 14s）
t_start = 0;
t_end = 20;
idx = data.Time >= t_start & data.Time <= t_end;

% 判断是否真的找到了数据，防止报错
if sum(idx) == 0
    error('在 %d 秒到 %d 秒之间没有找到数据，请检查时间范围！', t_start, t_end);
end
% ======================================================

% 5. 提取绘图所需数据
t = data.Time(idx);
y1 = data.ArmAngle(idx);
y2 = data.ArmVelocity(idx);
y3 = data.PendulumAngle(idx);
y4 = data.PendulumVelocity(idx);
y5 = data.debug_unfiltered_u(idx); % 提取新增的第5个变量
y6 = data.raw_control(idx); % 提取新增的第6个变量
y7 = data.raw_arm_angle(idx); % 提取新增的第6个变量
y8 = data.raw_pend_angle(idx); % 提取新增的第6个变量

% 4. 绘制五个子图 (使用 2x2 排版，也可改为 4x1)
figure;

% 第一个子图：Arm Angle
subplot(3, 2, 1);
plot(t, y1, 'b');
title(newNames{3}); 
xlabel('Time (s)');
ylabel('rad');
grid on;
xlim([t_start t_end]);

% 第二个子图：Pendulum Angle
subplot(3, 2, 2);
plot(t, y3, 'g');
title(newNames{5});
xlabel('Time (s)');
ylabel('rad');
grid on;
xlim([t_start t_end]);

% 第三个子图：Arm Velocity
subplot(3, 2, [3]);
plot(t, y2, 'r');
title(newNames{4});
xlabel('Time (s)');
ylabel('rad/s');
grid on;
xlim([t_start t_end]);

% 第四个子图：Pendulum Velocity
subplot(3, 2, 4);
plot(t, y4, 'k');
title(newNames{6});
xlabel('Time (s)');
ylabel('rad/s');
grid on;
xlim([t_start t_end]);

% 第五个子图：Raw Control (新增)
subplot(3, 2, [5, 6]);
plot(t, y6, 'm'); % 'm' 为品红色线条
title(newNames{2});
xlabel('Time (s)');
ylabel('V'); % 根据实际物理单位修改
grid on;
xlim([t_start t_end]);

% % 第六个子图：Raw Control (新增)
% subplot(3, 2, 6);
% plot(t, y8, 'm'); % 'm' 为品红色线条
% title(newNames{9});
% xlabel('Time (s)');
% ylabel('V'); % 根据实际物理单位修改
% grid on;
% xlim([t_start t_end]);