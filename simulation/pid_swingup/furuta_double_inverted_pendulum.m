% Resources used: Dynamic Modeling and Simulation of a Rotational Inverted
% Pendulum (J L Duart et al 2017 J. Phys.: Conf. Ser. 792 012081)

% L_arm is the length of the arm
% I_arm is the moment of inertia of the arm

% lcm_1 is the distance between the arm-connected end of the pendulum and the
% pendulum's center of mass
% I_1 is the moment of inertia of the pendulum along its length (negligible moment of
% inertia assumed along the width and height of the pendulum)
% m_1 is the mass of the pendulum

% Friction terms include viscous and coulomb. 
% Viscous friction = viscous_friction_coefficient*velocity
% Coulomb friction = coulomb_friction_torque*sign(velocity) <- this sign
% function introduces discontinuity at v=0 and so is modelled by tanh(k*v)
% instead, where k is the smoothing gain
% f_v_arm is the viscous damping coefficient of the arm
% f_c_arm is the coulomb friction torque (F_n*coefficient) of the arm
% f_v_1 is the viscous damping coefficient of the pendulum
% f_c_1 is the coulomb friction torque (F_n*coefficient) of the pendulum
% k_fc_arm is the coulomb friction smoothing gain for the arm (used in tanh to produce a
% continuous sign(v_theta_arm) effect)
% k_fc_1 is the coulomb friction smoothing gain for the pendulum (used in tanh to produce a
% continuous sign(v_theta_1) effect)
syms L_arm I_arm lcom_1 I_1 m_1 g
syms L_1 lcom_2 I_2 m_2 % Extra parameters due to double pendulum
syms f_v_arm f_c_arm k_fc_arm
syms f_v_1 f_c_1 k_fc_1
syms f_v_2 f_c_2 k_fc_2
syms arm_torque
syms theta_arm(t) theta_1(t) theta_2(t)

% X and Y and Z positions of the end of arm
x_arm = L_arm*cos(theta_arm);
y_arm = L_arm*sin(theta_arm);
z_arm = 0;

% X and Y and Z positions of the pendulum 1 COM
x_1_com = x_arm - lcom_1*sin(theta_1)*sin(theta_arm);
y_1_com = y_arm + lcom_1*sin(theta_1)*cos(theta_arm);
z_1_com = z_arm + lcom_1*cos(theta_1);

% X and Y and Z positions of the pendulum 1 end
x_1_end = x_arm - L_1*sin(theta_1)*sin(theta_arm);
y_1_end = y_arm + L_1*sin(theta_1)*cos(theta_arm);
z_1_end = z_arm + L_1*cos(theta_1);

% X and Y and Z positions of the pendulum 2 COM
x_2_com = x_1_end - lcom_2*sin(theta_2)*sin(theta_arm);
y_2_com = y_1_end + lcom_2*sin(theta_2)*cos(theta_arm);
z_2_com = z_1_end + lcom_2*cos(theta_2);

% X and Y and Z velocities of the pendulum 1 COM
v_x1_com = diff(x_1_com);
v_y1_com = diff(y_1_com);
v_z1_com = diff(z_1_com);
% X and Y and Z velocities of the pendulum 2 COM
v_x2_com = diff(x_2_com);
v_y2_com = diff(y_2_com);
v_z2_com = diff(z_2_com);
% Angular velocities from theta_arm and theta_1 and theta_2
v_theta_arm = diff(theta_arm);
v_theta_1 = diff(theta_1);
v_theta_2 = diff(theta_2);
v_funcs = [v_theta_arm, v_theta_1, v_theta_2];
acc_funcs = [diff(v_theta_arm), diff(v_theta_1), diff(v_theta_2)];

% Translational KE of the pendulum 1
T_1_translational = (1/2)*m_1*(v_x1_com^2 + v_y1_com^2 + v_z1_com^2);
% Rotational KE of the pendulum 1
T_1_rotational = (1/2)*I_1*v_theta_1^2;

% Translational KE of the pendulum 2
T_2_translational = (1/2)*m_2*(v_x2_com^2 + v_y2_com^2 + v_z2_com^2);
% Rotational KE of the pendulum 2
T_2_rotational = (1/2)*I_2*v_theta_2^2;

% Rotational KE of the arm
T_arm_rotational = (1/2)*I_arm*v_theta_arm^2;

% Total KE
T_total = T_1_translational + T_1_rotational + T_2_translational + T_2_rotational + T_arm_rotational;

% PE of the pendulum 1 (setting the zero PE height to be the arm height)
P_1 = m_1*g*z_1_com;
% PE of the pendulum 2 (setting the zero PE height to be the arm height)
P_2 = m_2*g*z_2_com;

% Total PE (The arm moves in the horizontal plane and has zero PE)
P_total = P_1 + P_2;

% Lagrangian
L = T_total - P_total;

% EOM for the degrees of freedoms (theta_arm and theta_1 and theta_2)
% Extra forces such as friction/disturbances to be added here
eq_arm_LHS = diff(diff(L, v_theta_arm)) - diff(L, theta_arm);
eq_arm_RHS = arm_torque - f_v_arm*v_theta_arm - f_c_arm*tanh(k_fc_arm*v_theta_arm);
eq_1_LHS = diff(diff(L, v_theta_1)) - diff(L, theta_1);
eq_1_RHS = - f_v_1*v_theta_1 - f_c_1*tanh(k_fc_1*v_theta_1);
eq_2_LHS = diff(diff(L, v_theta_2)) - diff(L, theta_2);
eq_2_RHS = - f_v_2*v_theta_2 - f_c_2*tanh(k_fc_2*v_theta_2);

eq_arm = simplify(eq_arm_LHS == eq_arm_RHS);
eq_1 = simplify(eq_1_LHS == eq_1_RHS);
eq_2 = simplify(eq_2_LHS == eq_2_RHS);

% M = simplify(jacobian([eq_arm_LHS, eq_1_LHS] , acc_funcs))
% G = simplify(subs([eq_arm_LHS, eq_1_LHS], [v_funcs, acc_funcs], [0,0,0,0]))
% C_x = simplify([eq_arm_LHS; eq_1_LHS] - M*acc_funcs.' - G.')
% C = simplify(jacobian(C_x, v_funcs)) % TODO: C has an extra 2 due to differentiation of theta_dot^2 right now

% Define the accelerations as syms variables
syms a_theta_arm a_theta_1 a_theta_2

% Find the diff(v_theta_arm), diff(v_theta_1), diff(v_theta_2) terms in eq_arm, eq_1 and eq_2 and replace with a_theta_arm, a_theta_1, a_theta_2 correspondingly
acc_vars = [a_theta_arm, a_theta_1, a_theta_2];
eqs_sub = subs([eq_arm, eq_1, eq_2], acc_funcs, acc_vars);

% Solve for the acceleration terms
sol_acc = solve(eqs_sub, acc_vars);
a_theta_arm = simplify(sol_acc.a_theta_arm);
a_theta_1 = simplify(sol_acc.a_theta_1);
a_theta_2 = simplify(sol_acc.a_theta_2);

% S (state) is defined as [theta_arm; v_theta_arm; theta_1; v_theta_1; theta_2; v_theta_2]
% Define state derivative
dSdt = [v_theta_arm; a_theta_arm; v_theta_1; a_theta_1; v_theta_2; a_theta_2];

% Create symbolic vector to store the state
S_var = sym('S_var', [6 1]);

% Replace all S terms (syms funcs) in dSdt with S_var (syms variables)
% *Important*: Replace the higher order terms first: 
% e.g. if theta_arm is replaced first with S_var(1), 
% diff(theta_arm) becomes diff(S_var(1)) such that
% later MATLAB cannot find
% 'diff(theta_arm)' to replace with S_var(2)
dSdt = subs(dSdt, [diff(theta_arm), diff(theta_1), diff(theta_2), theta_arm, theta_1, theta_2], [S_var(2), S_var(4), S_var(6), S_var(1), S_var(3), S_var(5)]);

% Obtain A and B matrices in state-space equation, linearised about the
% equilibrium point (arm position at 0, no arm velocity, upright position,
% no pendulum velocity)
equilibrium = [0; 0; 0; 0; 0; 0];
A_mat = jacobian(dSdt, S_var);
A_linear = simplify(subs(A_mat, S_var, equilibrium));
B_mat = jacobian(dSdt, arm_torque);
B_linear = simplify(subs(B_mat, S_var, equilibrium));

% % % Define parameters used in the functions below
phys_param_double = [L_arm, I_arm, L_1, lcom_1, I_1, m_1, lcom_2, I_2, m_2, g];
fric_arm = [f_v_arm, f_c_arm, k_fc_arm];
fric_1 = [f_v_1, f_c_1, k_fc_1];
fric_2 = [f_v_2, f_c_2, k_fc_2];

% Create a file containing the functions to be used in simulation
% This function outputs dSdt given inputs t, S_var, arm_torque, phys_param_double,
% fric_arm, fric_1, fric_2
matlabFunction(dSdt, 'File', 'functions/furuta_double_state_derivative', 'Vars', {t, S_var, arm_torque, phys_param_double, fric_arm, fric_1, fric_2});
% This function outputs A_linear and B_linear given inputs t, phys_param_double,
% fric_arm, fric_1, fric_2
matlabFunction(A_linear, B_linear, 'File', 'functions/furuta_double_A_B_matrices', 'Vars', {t, phys_param_double, fric_arm, fric_1, fric_2});