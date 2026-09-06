function [Lf_psi1, Lg_psi1, alpha2, psi1] = CBF_Constraint(x, params)
   % Extract states
    phi   = x(1);
    theta = x(2);
    p     = x(4);
    q     = x(5);
    r     = x(6);

    omega = [p; q; r];

    % Parameters
    I       = params.I;
    B_alloc = params.B_alloc;
    theta_max = params.theta_max;  

    alpha1 = params.alpha1;   
    alpha2 = params.alpha2;   

    h = theta_max - theta;

    J = euler_rate_matrix(phi, theta);
    J2 = J(2,:);        
    theta_dot = J2 * omega;

  
    psi0 = h;

    psi1 = -theta_dot + alpha1 * psi0;


    gyroTerm = I \ ( cross(omega, I*omega) );  

    Lf_psi1 = J2 * gyroTerm + alpha1 * theta_dot;


    Lg_psi1 = -J2 * (I \ B_alloc);
end

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