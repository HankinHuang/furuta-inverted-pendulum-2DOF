% Code for plotting the data obtained from simulation
% furuta_single_inverted_pendulum_model.slx
% Note that this code assumes To Workspace block is named plotData
% Make sure to set the parameter sizes in the get_dSdt MATLAB Function
% block Edit Data section

% Extract parameters from simulation output
t = out.plotData.state.Time;
S = out.plotData.state.Data;
L_arm = out.plotData.L_arm.Data(1);
L_1 = out.plotData.L_1.Data(1);
lcom_1 = out.plotData.lcom_1.Data(1);

% Initialise the animation and state variables log figures
[hArm, hPend1, hCOM1, hPend1Ref0, hPend1Label0, hPend1Ref90, hPend1Label90, hPend1Ref180, hPend1Label180, hPend1Ref270, hPend1Label270, hDataBox] = init_animation(L_arm, L_1);
[h_log1, h_log2, h_log3, h_log4] = init_state_log(t, S);

% Loop through the simulation output and update plot
for i = 1:length(t)
    theta_arm = S(i, 1);
    v_theta_arm = S(i, 2);
    theta_1   = S(i, 3);
    v_theta_1 = S(i, 4);

    theta_arm_deg = rad2deg(theta_arm);
    theta_1_deg = rad2deg(theta_1);

    theta_arm_deg_wrap = wrapTo180(theta_arm_deg);
    theta_1_deg_wrap = wrapTo180(theta_1_deg);

    current_t = t(i);

    update_animation(hArm, hPend1, hCOM1, ...
                    hPend1Ref0, hPend1Label0, hPend1Ref90, hPend1Label90, hPend1Ref180, hPend1Label180, hPend1Ref270, hPend1Label270, hDataBox, ...
                    current_t, ...
                    L_arm, L_1, lcom_1, ...
                    theta_arm, theta_arm_deg_wrap, v_theta_arm, ...
                    theta_1, theta_1_deg_wrap, v_theta_1)
    

    update_state_log(h_log1, h_log2, h_log3, h_log4, ...
                    current_t, theta_arm_deg_wrap, v_theta_arm, theta_1_deg_wrap, v_theta_1);

    drawnow limitrate;
end

function update_animation(hArm, hPend1, hCOM1, ...
                        hPend1Ref0, hPend1Label0, hPend1Ref90, hPend1Label90, hPend1Ref180, hPend1Label180, hPend1Ref270, hPend1Label270, hDataBox, ...
                        current_t, ...
                        L_arm, L_1, lcom_1, ...
                        theta_arm, theta_arm_deg_wrap, v_theta_arm, ...
                        theta_1, theta_1_deg_wrap, v_theta_1)
    % Calculate Coordinates
    % Arm end point
    x_arm = L_arm * cos(theta_arm);
    y_arm = L_arm * sin(theta_arm);
    z_arm = 0;
    
    % Pendulum end point
    x_1 = x_arm - L_1 * sin(theta_1) * sin(theta_arm);
    y_1 = y_arm + L_1 * sin(theta_1) * cos(theta_arm);
    z_1 = z_arm + L_1 * cos(theta_1);

    % Pendulum COM point
    x_com1 = x_arm - lcom_1 * sin(theta_1) * sin(theta_arm);
    y_com1 = y_arm + lcom_1 * sin(theta_1) * cos(theta_arm);
    z_com1 = z_arm + lcom_1 * cos(theta_1);

    set(hArm,  'XData', [0 x_arm], 'YData', [0 y_arm], 'ZData', [0 z_arm]);
    set(hPend1, 'XData', [x_arm x_1], 'YData', [y_arm y_1], 'ZData', [z_arm z_1]);
    set(hCOM1,  'XData', x_com1, 'YData', y_com1, 'ZData', z_com1);
    data_str = sprintf(['Time: %.2f s\n' ...
                        'Arm Angle: %.2f rad (%.2f deg)\n' ...
                        'Arm Vel:   %.2f rad/s\n' ...
                        'Pend 1 Angle: %.2f rad (%.2f deg)\n' ...
                        'Pend 1 Vel:   %.2f rad/s'], ...
                        current_t, theta_arm, theta_arm_deg_wrap, v_theta_arm, theta_1, theta_1_deg_wrap, v_theta_1);
    set(hDataBox, 'String', data_str);

    z_Pend1_ref0 = z_arm + L_1; 
    z_Pend1_ref180 = z_arm - L_1; 
    x_Pend1_ref90 = x_arm - L_1 * sin(theta_arm);
    y_Pend1_ref90 = y_arm + L_1 * cos(theta_arm);
    x_Pend1_ref270 = x_arm + L_1 * sin(theta_arm);
    y_Pend1_ref270 = y_arm - L_1 * cos(theta_arm);

    set(hPend1Ref0, 'XData', [x_arm x_arm], 'YData', [y_arm y_arm], 'ZData', [z_arm z_Pend1_ref0]);
    set(hPend1Ref90, 'XData', [x_arm x_Pend1_ref90], ...
            'YData', [y_arm y_Pend1_ref90], ...
            'ZData', [z_arm z_arm]);
    set(hPend1Ref180, 'XData', [x_arm x_arm], 'YData', [y_arm y_arm], 'ZData', [z_arm z_Pend1_ref180]);
    set(hPend1Ref270, 'XData', [x_arm x_Pend1_ref270], ...
            'YData', [y_arm y_Pend1_ref270], ...
            'ZData', [z_arm z_arm]);

    set(hPend1Label0, 'Position', [x_arm, y_arm, z_Pend1_ref0 + (L_1 * 0.01)]);
    set(hPend1Label180, 'Position', [x_arm, y_arm, z_Pend1_ref180 - (L_1 * 0.01)]);
    set(hPend1Label90, 'Position', [x_Pend1_ref90, y_Pend1_ref90, z_arm + (L_1 * 0.01)]);
    set(hPend1Label270, 'Position', [x_Pend1_ref270, y_Pend1_ref270, z_arm + (L_1 * 0.01)]);
end

function update_state_log(h_log1, h_log2, h_log3, h_log4, ...
                        current_t, theta_arm_deg_wrap, v_theta_arm, theta_1_deg_wrap, v_theta_1)
    addpoints(h_log1, current_t, theta_arm_deg_wrap); % Theta Arm
    addpoints(h_log2, current_t, v_theta_arm); % Theta Arm Dot (Velocity)
    addpoints(h_log3, current_t, theta_1_deg_wrap); % Theta 1
    addpoints(h_log4, current_t, v_theta_1); % Theta 1 Dot (Velocity)
end

function [hArm, hPend1, hCOM1, hPend1Ref0, hPend1Label0, hPend1Ref90, hPend1Label90, hPend1Ref180, hPend1Label180, hPend1Ref270, hPend1Label270, hDataBox] = init_animation(L_arm, L_1)
    % By default, simulink will try compiling the function into C code but
    % uiconrol is a MATLAB function and does not have a C equivalent. This
    % line is telling the compiler not to translate uicontrol into C and
    % call the MATLAB engine to execute it instead
    coder.extrinsic('uicontrol'); 
    
    f = figure('Name', 'Furuta Pendulum 3D Playback', 'NumberTitle', 'off', 'WindowStyle', 'normal');
    view(3); grid on; axis equal;
    limit = L_arm + L_1;
    axis([-limit limit -limit limit -limit limit]);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    hold on;
    
    hArm = plot3([0 0], [0 0], [0 0], 'r-', 'LineWidth', 3, 'DisplayName', 'Arm');
    hPend1 = plot3([0 0], [0 0], [0 0], 'b-', 'LineWidth', 3, 'DisplayName', 'Pendulum 1');
    hCOM1 = plot3(0, 0, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'COM of Pendulum 1');
    
    hPend1Ref0  = plot3([0 0], [0 0], [0 0], 'k--', 'LineWidth', 0.5, 'DisplayName', 'Pendulum 1: 0 deg');
    hPend1Ref90 = plot3([0 0], [0 0], [0 0], 'm--', 'LineWidth', 0.5, 'DisplayName', 'Pendulum 1: 90 deg');
    hPend1Ref180  = plot3([0 0], [0 0], [0 0], 'g--', 'LineWidth', 0.5, 'DisplayName', 'Pendulum 1: 180 deg');
    hPend1Ref270 = plot3([0 0], [0 0], [0 0], 'c--', 'LineWidth', 0.5, 'DisplayName', 'Pendulum 1: 270 deg');
    
    hPend1Label0 = text(0, 0, 0, ' \theta_1 = 0 deg', 'FontWeight', 'bold', 'FontSize', 9);
    hPend1Label90 = text(0, 0, 0, ' \theta_1 = 90 deg', 'FontWeight', 'bold', 'FontSize', 9);
    hPend1Label180 = text(0, 0, 0, ' \theta_1 = 180 deg', 'FontWeight', 'bold', 'FontSize', 9);
    hPend1Label270 = text(0, 0, 0, ' \theta_1 = 270 deg', 'FontWeight', 'bold', 'FontSize', 9);
    
    plot3(0, 0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', [0.2 0.2 0.2], 'DisplayName', 'Motor');
    plot3([-limit limit], [0 0], [0 0], 'k:', 'LineWidth', 0.5, 'DisplayName', 'X-axis'); 
    plot3([0 0], [-limit limit], [0 0], 'k:', 'LineWidth', 0.5, 'DisplayName', 'Y-axis'); 
    
    text(limit, 0, 0, ' \theta_{arm} = 0 deg', 'FontWeight', 'bold');
    text(0, limit, 0, ' \theta_{arm} = 90 deg', 'FontWeight', 'bold');
    
    legend('Location', 'northeastoutside');
    
    hDataBox = uicontrol('Style', 'text', ...
                      'HorizontalAlignment', 'left', ...
                      'FontSize', 10, ...
                      'BackgroundColor', [0.95 0.95 0.95], ...
                      'Position', [10, 10, 300, 100], ... % Bottom left corner
                      'String', 'Initializing...');
end

function [h_log1, h_log2, h_log3, h_log4] = init_state_log(t, S)
    f_log = figure('Name', 'State Variables Log', 'NumberTitle', 'off', 'WindowStyle', 'normal');
    
    total_duration = t(end);
    
    % Subplot 1: Arm Angle (Theta Arm)
    ax1 = subplot(4,1,1);
    h_log1 = animatedline('Color', 'k', 'LineWidth', 1.5);
    title('Arm Angle (wrapped to [-180, 180])');
    ylabel('deg');
    grid on;
    xlim(ax1, [0, total_duration]);
    ylim(ax1, [-270, 270]);
    
    % Subplot 2: Arm Velocity (Theta Arm Dot)
    ax2 = subplot(4,1,2);
    h_log2 = animatedline('Color', 'b', 'LineWidth', 1.5);
    title('Arm Velocity');
    ylabel('rad/s');
    grid on;
    xlim(ax2, [0, total_duration]);
    ylim(ax2, [min(S(:, 2)), max(S(:, 2))]);
    
    % Subplot 3: Pendulum 1 Angle (Theta 1)
    ax3 = subplot(4,1,3);
    h_log3 = animatedline('Color', 'r', 'LineWidth', 1.5);
    title('Pendulum 1 Angle (wrapped to [-180, 180])');
    ylabel('deg');
    grid on;
    xlim(ax3, [0, total_duration]);
    ylim(ax3, [-270, 270]);
    
    % Subplot 4: Pendulum 1 Velocity (Theta 1 Dot)
    ax4 = subplot(4,1,4);
    h_log4 = animatedline('Color', 'g', 'LineWidth', 1.5);
    title('Pendulum 1 Velocity');
    ylabel('rad/s');
    xlabel('Time (s)');
    grid on;
    xlim(ax4, [0, total_duration]);
    ylim(ax4, [min(S(:, 4)), max(S(:, 4))]);
    
    % Link x-axes so zooming one zooms all
    linkaxes([ax1, ax2, ax3, ax4], 'x');

end