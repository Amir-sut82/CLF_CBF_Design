function u_star = CLF_CBF_Controller(x, params)

    p_penalty = params.p_penalty;   
    gamma     = params.gamma;
    I         = params.I;
    B_alloc   = params.B_alloc;
    P         = params.P;

    % Extract states
    phi   = x(1);
    theta = x(2);
    psi   = x(3);
    p     = x(4);
    q     = x(5);
    r     = x(6);

    omega = [p; q; r];
    y     = [phi; theta - deg2rad(30); psi];

    J = euler_rate_matrix(phi, theta);

    y_dot = J * omega;

    eta = [y; y_dot];

    V = eta' * P * eta;

    if V < 1e-8
        u_star = zeros(4,1);
        return;
    end

    Jdot = compute_Jdot(phi, theta, omega);
    tau_gyro        = -cross(omega, I * omega);   
    omega_dot_drift = I \ tau_gyro;

    % f and g in eta-space
    f_omega = Jdot * omega + J * omega_dot_drift;  
    A_eta   = J * (I \ B_alloc);                  

    f_eta = [y_dot;   f_omega];   
    g_eta = [zeros(3,4); A_eta]; 

    dV_deta = 2 * eta' * P;   

    LfV = dV_deta * f_eta;   
    LgV = dV_deta * g_eta;    

%% CBF: Pitch HOCBF via CBF_Constraint
    [Lf_psi1, Lg_psi1, alpha2, psi1] = CBF_Constraint(x, params);
    A_cbf_row = -Lg_psi1;                        
    b_cbf_val =  Lf_psi1 + alpha2 * psi1;       

    A_cbf = [A_cbf_row, 0];                      


%% Unified QP

    H   = 2 * diag([1, 1, 1, 1, p_penalty]);
    fqp = zeros(5, 1);

    A_clf = [LgV, -1];          
    b_clf = -LfV - gamma * V;   

    Aineq = [A_clf; A_cbf];
    bineq = [b_clf; b_cbf_val];

    if isfield(params, 'u_min'), umin = params.u_min; else, umin = -0.01; end
    if isfield(params, 'u_max'), umax = params.u_max; else, umax =  0.01; end

    lb = [umin * ones(4,1);  0  ];   
    ub = [umax * ones(4,1); Inf];

    opts = optimoptions('quadprog', 'Display', 'off', 'MaxIterations', 200);

    z = quadprog(H, fqp, Aineq, bineq, [], [], lb, ub, [], opts);

    if isempty(z)
        warning('CLF–CBF QP failed. Returning zero control.');
        u_star = zeros(4,1);
    else
        u_star = z(1:4);
    end

end

%% Local helpers 

function J = euler_rate_matrix(phi, theta)

    cphi = cos(phi);  sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);

    eps_th = 1e-6;
    if abs(cth) < eps_th
        cth = eps_th * sign(cth + eps_th);
    end

    J = [1,  sphi*sth/cth,   cphi*sth/cth;
         0,  cphi,           -sphi;
         0,  sphi/cth,        cphi/cth];
end

function Jdot = compute_Jdot(phi, theta, omega)
    J = euler_rate_matrix(phi, theta);
    euler_dot = J * omega;

    phi_dot   = euler_dot(1);
    theta_dot = euler_dot(2);

    sphi = sin(phi);  cphi = cos(phi);
    cth  = cos(theta); sth  = sin(theta);

    eps_th = 1e-6;
    if abs(cth) < eps_th
        cth = eps_th * sign(cth + eps_th);
    end

    tan_th  = sth / cth;
    sec_th  = 1   / cth;
    sec2_th = sec_th^2;

    dJ_dphi = [0,  cphi*tan_th,           -sphi*tan_th;
               0, -sphi,                  -cphi;
               0,  cphi*sec_th,           -sphi*sec_th];

    dJ_dtheta = [0,  sphi*sec2_th,          cphi*sec2_th;
                 0,  0,                      0;
                 0,  sphi*sec_th*tan_th,     cphi*sec_th*tan_th];

    Jdot = dJ_dphi * phi_dot + dJ_dtheta * theta_dot;
end