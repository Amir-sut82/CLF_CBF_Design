function [f, g] = Dynamics(x)
% DYNAMICS
% x = [phi; theta; psi; p; q; r]

    params = System_Parameters();

    I = params.I;
    B_alloc = params.B_alloc;

    % States
    phi   = x(1);
    theta = x(2);
    psi = x(3); 
    p     = x(4);
    q     = x(5);
    r     = x(6);

    omega = [p; q; r];

    % Euler kinematics matrix
    J = euler_rate_matrix(phi, theta);

    % Drift term
    euler_dot_drift = J * omega;

    tau_gyro = -cross(omega, I * omega);
    omega_dot_drift = I \ tau_gyro;

    f = [euler_dot_drift;
         omega_dot_drift];

    % Input matrix
    g = [zeros(3,4);
         I \ B_alloc];
end

%% ===== local helper =====
function J = euler_rate_matrix(phi, theta)
    cphi = cos(phi);
    sphi = sin(phi);
    cth  = cos(theta);
    sth  = sin(theta);

    % protect against singularity
    eps_th = 1e-6;
    if abs(cth) < eps_th
        cth = sign(cth + eps_th) * eps_th;
    end

    J = [1,  sphi*sth/cth,   cphi*sth/cth;
         0,  cphi,          -sphi;
         0,  sphi/cth,       cphi/cth];
end
