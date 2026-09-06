%% Main.m  — Bi-Copter Attitude Control
clear; clc; close all;

params = System_Parameters();

tspan = [0 60];
gamma = params.gamma;

% ---------------------------------------------------------------
% USER CONTROLLER SELECTION
% ---------------------------------------------------------------
control_mode = 'Unified';   % 'CLF_Only' | 'Decoupled' | 'Unified'

% CLF-QP type
%   'unconstrained' | 'constrained_hard' | 'constrained_slack'
qp_type  = 'unconstrained';
CBF_type = 'Constrained_CBF';   % only relevant in Decoupled mode

% Initial state [phi; theta; psi; p; q; r]  (rad)
x0 = deg2rad([10; 0; 30; 0; 0; 0]);

[t, x] = ode45( ...
    @(t, x) closed_loop_dynamics(t, x, gamma, qp_type, CBF_type, params, control_mode), ...
    tspan, x0);


u_nom_hist     = zeros(length(t), 4);
u_applied_hist = zeros(length(t), 4);

for i = 1:length(t)
    x_i   = x(i,:)';
    u_nom = CLF_Controller(x_i, gamma, qp_type);
    u_nom_hist(i,:) = u_nom';

    switch control_mode
        case 'CLF_Only'
            u_applied = u_nom;                                         
        case 'Decoupled'
            u_applied = Safety_Filter(x_i, u_nom, params, CBF_type);
        case 'Unified'
            u_applied = CLF_CBF_Controller(x_i, params);
        otherwise
            error('Invalid control_mode: ''%s''', control_mode);
    end

    u_applied_hist(i,:) = u_applied';
end


% Plot
Plot_Results(t, x, u_applied_hist, u_nom_hist, control_mode, params);


%% ================= LOCAL FUNCTIONS =================

function dx = closed_loop_dynamics(~, x, gamma, qp_type, CBF_type, params, control_mode)

    I       = params.I;
    B_alloc = params.B_alloc;

    switch control_mode
        case 'CLF_Only'
            u = CLF_Controller(x, gamma, qp_type);
        case 'Decoupled'
            u_nom = CLF_Controller(x, gamma, qp_type);
            u     = Safety_Filter(x, u_nom, params, CBF_type);
        case 'Unified'
            u = CLF_CBF_Controller(x, params);
        otherwise
            error('Invalid control_mode: ''%s''', control_mode);
    end

    phi   = x(1);
    theta = x(2);
    p     = x(4);
    q     = x(5);
    r     = x(6);

    omega     = [p; q; r];
    J         = euler_rate_matrix(phi, theta);
    euler_dot = J * omega;

    tau_gyro  = -cross(omega, I * omega);
    omega_dot = I \ (tau_gyro + B_alloc * u);

    dx = [euler_dot; omega_dot];
end


function J = euler_rate_matrix(phi, theta)
    cphi = cos(phi);  sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);

    eps_th = 1e-6;
    if abs(cth) < eps_th
        cth = sign(cth + eps_th) * eps_th;
    end

    J = [1,  sphi*sth/cth,   cphi*sth/cth;
         0,  cphi,           -sphi;
         0,  sphi/cth,        cphi/cth];
end