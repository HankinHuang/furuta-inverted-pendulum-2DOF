% 假设 A 和 B 是你用之前脚本算出来的 4x4 和 4x1 数字矩阵
% 假设 K_ideal 是你用 lqr(A, B, Q, R) 算出来的原始增益 [k1, k2, k3, k4]

k1 = K_voltage(1); k2 = K_voltage(2); k3 = K_voltage(3);

% 创建一个图形窗口
figure;
hold on;
grid on;
title('The trajectory of closed-loop poles as K_4 varies (Root Migration)');
xlabel('Real Axis - Determining the convergence rate');
ylabel('Imaginary Axis - Determining the oscillation frequency');

% 让 K4 从理想值（比如 -2.0）变化到你设定的 -0.6，甚至到 0
K4_range = linspace(K_voltage(4), 0, 100); 

for i = 1:length(K4_range)
    K_test = [k1, k2, k3, K4_range(i)];
    A_cl = A_new - B_new * K_test;
    poles = eig(A_cl); % 计算闭环极点
    
    % 画出极点，可以用颜色深浅表示变化过程
plot(real(poles), imag(poles), 'x', 'Color', [0.6 0.6 0.6], 'MarkerSize', 5);
end

% 特别标出 K4 = -0.6 时的极点位置
K_soft = [k1, k2, -17.1437, -1.3495];
poles_soft = eig(A_new - B_new * K_soft);
plot(real(poles_soft), imag(poles_soft), 'ro', 'MarkerSize', 10, 'LineWidth', 2.5);
damp(A_new - B_new * K_soft)

% 重写图例，确保对应准确
legend('x is the trajectory of pole movement (K_4 tends from the ideal value to 0)', 'The pole setting for Soft-LQR is (K₄ = -0.515)');