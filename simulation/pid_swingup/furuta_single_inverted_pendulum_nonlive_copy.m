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
syms f_v_arm f_c_arm k_fc_arm
syms f_v_1 f_c_1 k_fc_1
syms arm_torque
syms theta_arm(t) theta_1(t)
%%
% X and Y and Z positions of the end of arm
x_arm = L_arm*cos(theta_arm);
y_arm = L_arm*sin(theta_arm);
z_arm = 0;

% X and Y and Z positions of the pendulum COM
x_1_com = x_arm - lcom_1*sin(theta_1)*sin(theta_arm);
y_1_com = y_arm + lcom_1*sin(theta_1)*cos(theta_arm);
z_1_com = z_arm + lcom_1*cos(theta_1);

% X and Y and Z velocities of the pendulum COM
v_x1_com = diff(x_1_com);
v_y1_com = diff(y_1_com);
v_z1_com = diff(z_1_com);
% Angular velocities from theta_arm and theta_1
v_theta_arm = diff(theta_arm);
v_theta_1 = diff(theta_1);
v_funcs = [v_theta_arm, v_theta_1];
acc_funcs = [diff(v_theta_arm), diff(v_theta_1)];

% Translational KE of the pendulum
T_1_translational = (1/2)*m_1*(v_x1_com^2 + v_y1_com^2 + v_z1_com^2);
% Rotational KE of the pendulum
T_1_rotational = (1/2)*I_1*v_theta_1^2;
% Rotational KE of the arm
T_arm_rotational = (1/2)*I_arm*v_theta_arm^2;

% Total KE
T_total = T_1_translational + T_1_rotational + T_arm_rotational;

% PE of the pendulum (setting the zero PE height to be the arm height)
P_1 = m_1*g*z_1_com;

% Total PE (The arm moves in the horizontal plane and has zero PE)
P_total = P_1;

% Lagrangian
L = T_total - P_total;

% EOM for the degrees of freedoms (theta_arm and theta_1)
% Extra forces such as friction/disturbances to be added here
eq_arm_LHS = diff(diff(L, v_theta_arm)) - diff(L, theta_arm);
eq_arm_RHS = arm_torque - f_v_arm*v_theta_arm - f_c_arm*tanh(k_fc_arm*v_theta_arm);
eq_1_LHS = diff(diff(L, v_theta_1)) - diff(L, theta_1);
eq_1_RHS = - f_v_1*v_theta_1 - f_c_1*tanh(k_fc_1*v_theta_1);
eq_arm = simplify(eq_arm_LHS == eq_arm_RHS);
eq_1 = simplify(eq_1_LHS == eq_1_RHS)

% M = simplify(jacobian([eq_arm_LHS, eq_1_LHS] , acc_funcs))
% G = simplify(subs([eq_arm_LHS, eq_1_LHS], [v_funcs, acc_funcs], [0,0,0,0]))
% C_x = simplify([eq_arm_LHS; eq_1_LHS] - M*acc_funcs.' - G.')
% C = simplify(jacobian(C_x, v_funcs)) % TODO: C has an extra 2 due to differentiation of theta_dot^2 right now

% Define the accelerations as syms variables
syms a_theta_arm a_theta_1

% Find the diff(v_theta_arm), diff(v_theta_1) terms in eq_arm, eq_1, and replace with a_theta_arm, a_theta_1 correspondingly
acc_vars = [a_theta_arm, a_theta_1];
eqs_sub = subs([eq_arm, eq_1], acc_funcs, acc_vars);

% Solve for the acceleration terms
sol_acc = solve(eqs_sub, acc_vars);
a_theta_arm = simplify(sol_acc.a_theta_arm);
a_theta_1 = simplify(sol_acc.a_theta_1);

% S (state) is defined as [theta_arm; v_theta_arm; theta_1; v_theta_1]
% Define state derivative
dSdt = [v_theta_arm; a_theta_arm; v_theta_1; a_theta_1];

% Create symbolic vector to store the state
S_var = sym('S_var', [4 1])

% Replace all S terms (syms funcs) in dSdt with S_var (syms variables)
% *Important*: Replace the higher order terms first: 
% e.g. if theta_arm is replaced first with S_var(1), 
% diff(theta_arm) becomes diff(S_var(1)) such that
% later MATLAB cannot find
% 'diff(theta_arm)' to replace with S_var(2)
dSdt = subs(dSdt, [diff(theta_arm), diff(theta_1), theta_arm, theta_1], [S_var(2), S_var(4), S_var(1), S_var(3)]);

% Obtain A and B matrices in state-space equation, linearised about the
% equilibrium point (arm position at 0, no arm velocity, upright position,
% no pendulum velocity)
equilibrium = [0; 0; 0 ;0];
A_mat = simplify(jacobian(dSdt, S_var))
A_linear = simplify(subs(A_mat, S_var, equilibrium))
B_mat = simplify(jacobian(dSdt, arm_torque))
B_linear = simplify(subs(B_mat, S_var, equilibrium))

% % % Define parameters used in the functions below
phys_param = [L_arm, I_arm, lcom_1, I_1, m_1, g];
fric_arm = [f_v_arm, f_c_arm, k_fc_arm];
fric_1 = [f_v_1, f_c_1, k_fc_1];

% Create a file containing the functions to be used in simulation
% This function outputs dSdt given inputs t, S_var, arm_torque, phys_param, fric_arm, fric_1
matlabFunction(dSdt, 'File', 'functions/furuta_single_state_derivative', 'Vars', {t, S_var, arm_torque, phys_param, fric_arm, fric_1});
% This function outputs A_linear and B_linear given inputs t, phys_param, fric_arm, fric_1
matlabFunction(A_linear, B_linear, 'File', 'functions/furuta_single_A_B_matrices', 'Vars', {t, phys_param, fric_arm, fric_1});