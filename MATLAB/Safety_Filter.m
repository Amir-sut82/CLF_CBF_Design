function u = Safety_Filter(x, u_nom, params, CBF_type)
    

    [Lf_psi1, Lg_psi1, alpha2, psi1] = CBF_Constraint(x, params);
  
    A_cbf = -Lg_psi1;                % convert to standard quadprog form A u ≤ b
    b_cbf = Lf_psi1 + alpha2 * psi1;

    H = 2 * eye(4);
    f = -2 * u_nom;


    switch CBF_type

        case 'Constrained_CBF'
    if isfield(params,'u_min')
        lb = params.u_min * ones(4,1);
    else
        lb = -inf(4,1);
    end

    if isfield(params,'u_max')
        ub = params.u_max * ones(4,1);
    else
        ub = inf(4,1);
    end

    options = optimoptions('quadprog','Display','off');
    [u,~,exitflag] = quadprog(H,f,A_cbf,b_cbf,[],[],lb,ub,u_nom,options);

    if exitflag <= 0 || isempty(u)
        warning('Safety filter: QP failed → using nominal control.');
        u = u_nom;
    end

        case 'Unconstrained_CBF'
            options = optimoptions('quadprog','Display','off');
            [u,~,exitflag] = quadprog(H,f,A_cbf,b_cbf,[],[],[],[],u_nom,options);
        
            if exitflag <= 0 || isempty(u)
                warning('Safety filter: QP failed → using nominal control.');
                u = u_nom;
            end
    end
end