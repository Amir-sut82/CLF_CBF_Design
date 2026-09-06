function params = System_Parameters()
% SYSTEM_PARAMETERS

    params.m = 0.486;      % kg
    params.g = 9.81;       % m/s^2

   
    params.theta_max = deg2rad(20);    

    % HOCBF gains
    params.alpha1 = 5;
    params.alpha2 = 5;

    params.gamma = 0.5;

    params.p_penalty = 1e4;

    % Actuator bounds (used by CLF_CBF_Controller and Safety_Filter)
    params.u_min = -0.1;   % N·m  (rotor moment lower bound)
    params.u_max =  0.1;   % N·m  (rotor moment upper bound)

    % Inertia tensor
    params.Ixx = 0.0043;
    params.Iyy = 0.0142;
    params.Izz = 0.0176;
    params.Ixz = 0.0001;

    params.I = [ params.Ixx,  0,           -params.Ixz;
                 0,            params.Iyy,   0;
                -params.Ixz,  0,            params.Izz ];

    params.dcg = 0.175;        % m   (horizontal arm length to rotor)
    params.hcg = 0.085;        % m   (vertical offset to rotor)
    params.b   = 3.13e-5;      % N·s^2  (thrust coefficient)
    params.d   = 1.2e-6;       % N·m·s^2  (drag coefficient)
    params.d_over_b = params.d / params.b;

   
    params.B_alloc = [0,  0,            params.dcg,      params.d_over_b;
                      0,  params.hcg,   0,               0;
                      0,  0,            params.d_over_b, -params.dcg    ];

    % CLF design:
    params.Kp = 4 * eye(3);
    params.Kd = 4 * eye(3);

    F = [zeros(3), eye(3);
         zeros(3), zeros(3)];
    G = [zeros(3);
         eye(3)];
    K   = [params.Kp, params.Kd];
    Acl = F - G * K;

    Q = 0.1 * eye(6);

    % Lyapunov matrix P
    params.P = lyap(Acl', Q);

    params.F   = F;
    params.G   = G;
    params.K   = K;
    params.Acl = Acl;
    params.Q   = Q;

end