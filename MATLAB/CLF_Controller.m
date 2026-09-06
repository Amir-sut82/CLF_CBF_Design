function u_stab = CLF_Controller(x, gamma, qp_type)
% CLF_CONTROLLER

    if nargin < 3
        qp_type = 'constrained_slack'; 
    end

    params = System_Parameters();

    I = params.I;
    B_alloc = params.B_alloc;
    P = params.P;

    % Extract states
    phi   = x(1);
    theta = x(2);
    psi   = x(3);
    p     = x(4);
    q     = x(5);
    r     = x(6);

    omega = [p; q; r];
    y = [phi; theta - deg2rad(30); psi];

    J = euler_rate_matrix(phi, theta);

    y_dot = J * omega;

    eta = [y; y_dot];

    % CLF
    V = eta' * P * eta;


    Jdot = compute_Jdot(phi, theta, omega);

    tau_gyro = -cross(omega, I * omega);
    omega_dot_drift = I \ tau_gyro;

    alpha = Jdot * omega + J * omega_dot_drift;   % 3x1
    A = J * (I \ B_alloc);                        % 3x4

    f_eta = [y_dot;
             alpha];

    g_eta = [zeros(3,4);
             A];


    dV_deta = 2 * eta' * P;   

    LfV = dV_deta * f_eta;    
    LgV = dV_deta * g_eta;   

    opts = optimoptions('quadprog', 'Display', 'off', 'MaxIterations', 100);

    if V < 1e-6
        u_stab = zeros(4,1);
        return;
    end

    u_max = 0.01;      % Maximum rotor moment (adjust as needed)
    u_min = -0.01;
    lb_u = u_min * ones(4,1);
    ub_u = u_max * ones(4,1);

    switch qp_type
        case 'unconstrained'
            H = 2 * eye(4);
            fqp = zeros(4,1);
            Aineq = LgV;                  
            bineq = -LfV - gamma * V;     
            
            u_stab = quadprog(H, fqp, Aineq, bineq, [], [], [], [], [], opts);

        case 'constrained_hard'
            H = 2 * eye(4);
            fqp = zeros(4,1);
            Aineq = LgV;                  
            bineq = -LfV - gamma * V;     
            
            u_stab = quadprog(H, fqp, Aineq, bineq, [], [], lb_u, ub_u, [], opts);

        case 'constrained_slack'
        
            w_slack = 1e6; 
            
            H = 2 * diag([1, 1, 1, 1, w_slack]); 
            fqp = zeros(5,1);
            
            Aineq = [LgV, -1]; 
            bineq = -LfV - gamma * V;
            
            lb = [lb_u; 0];    
            ub = [ub_u; Inf];
            
            z_stab = quadprog(H, fqp, Aineq, bineq, [], [], lb, ub, [], opts);
            
            if ~isempty(z_stab)
                u_stab = z_stab(1:4);
            else
                u_stab = [];
            end
            
        otherwise
            error('Unknown qp_type. Choose unconstrained, constrained_hard, or constrained_slack.');
    end

    if isempty(u_stab)
        u_stab = zeros(4,1);
    end
end

%% ===== local helpers =====
function J = euler_rate_matrix(phi, theta)
    cphi = cos(phi);
    sphi = sin(phi);
    cth  = cos(theta);
    sth  = sin(theta);

    eps_th = 1e-6;
    if abs(cth) < eps_th
        cth = sign(cth + eps_th) * eps_th;
    end

    J = [1,  sphi*sth/cth,   cphi*sth/cth;
         0,  cphi,          -sphi;
         0,  sphi/cth,       cphi/cth];
end

function Jdot = compute_Jdot(phi, theta, omega)
    p = omega(1);
    q = omega(2);
    r = omega(3);

    J = euler_rate_matrix(phi, theta);
    euler_dot = J * [p; q; r];

    phi_dot   = euler_dot(1);
    theta_dot = euler_dot(2);

    sphi = sin(phi);
    cphi = cos(phi);
    cth  = cos(theta);
    sth  = sin(theta);

    eps_th = 1e-6;
    if abs(cth) < eps_th
        cth = sign(cth + eps_th) * eps_th;
    end

    tan_th  = sth / cth;
    sec_th  = 1 / cth;
    sec2_th = sec_th^2;

    dJ_dphi = [0,  cphi*tan_th,      -sphi*tan_th;
               0, -sphi,             -cphi;
               0,  cphi*sec_th,      -sphi*sec_th];

    dJ_dtheta = [0,  sphi*sec2_th,             cphi*sec2_th;
                  0,  0,                       0;
                  0,  sphi*sec_th*tan_th,      cphi*sec_th*tan_th];

    Jdot = dJ_dphi * phi_dot + dJ_dtheta * theta_dot;
end
