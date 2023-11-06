function [Phi_x, Phi_u, objective,obj_h2,obj_hinf] = calculus_regret(sls, opt, Benchmark,verbose)

    vector_Phi_u = sdpvar(sls.m*opt.N_tf, sls.n, 'full');

    temp = eye(opt.T);
    Phi_u_tilda = kron(temp, vector_Phi_u(1:sls.m,:));
    for i = 1:(opt.N_tf - 1)
        temp = diag(ones(opt.T-i,1), -i);
        Phi_u_tilda = Phi_u_tilda + kron(temp, vector_Phi_u((i*sls.m + 1):((i + 1)*sls.m),:));
    end


    Phi_x_tilda = sls.P12*Phi_u_tilda + eye(sls.n*sls.T);
    temp = inv(sls.I - sls.Z*sls.A);
    Phi_u = Phi_u_tilda*temp;
    Phi_x = Phi_x_tilda*temp;

    Phi = [Phi_x; Phi_u];

    lambda = sdpvar(1, 1,'symmetric'); % Maximum eigenvalue to be minimized
    
    half_J_benchmark = chol(opt.C)*[Benchmark.x; Benchmark.u];
    % J_benchmark = [Benchmark.x; Benchmark.u]'*opt.C*[Benchmark.x; Benchmark.u];
    J_benchmark = half_J_benchmark'*half_J_benchmark;

    % Define the objective function
    objective = lambda;
   
    constraints = [];

    P = [eye((sls.n+sls.m)*opt.T) chol(opt.C)*[Phi_x; Phi_u];
    (chol(opt.C)*[Phi_x; Phi_u])' lambda*eye(sls.n*opt.T)+J_benchmark];

    constraints = [constraints, P >= 0];
    constraints = [constraints, lambda>=1e-3];
    constraints = [constraints, (Phi_u_tilda).*(~(sls.Vu_Sreal)) == 0];
    constraints = [constraints, (Phi_x_tilda).*(~(sls.Vx_Sreal)) == 0];


    % Solve the optimization problem
    options = sdpsettings('verbose', verbose, 'solver', 'mosek');
    % options = sdpsettings('verbose', verbose, 'solver', 'mosek','MSK_IPAR_NUM_THREADS', 30);
    sol = optimize(constraints, objective, options);
    if ~(sol.problem == 0)
        error("Error during Regret. Problem status: " +num2str(sol.problem) + " ( " + yalmiperror(sol.problem) + ")");
    end
    

    Phi_x = value(Phi_x); 
    Phi_u = value(Phi_u);
    
    objective = value(objective);
    obj_h2 = norm(chol(opt.C)*[Phi_x; Phi_u], 'fro')^2;                   % Extract the H2-optimal   cost incurred by the unconstrained clairvoyant controller
    obj_hinf = norm(chol(opt.C)*[Phi_x; Phi_u], 2)^2;                     % Compute the Hinf-optimal cost incurred by the unconstrained clairvoyant controller  
end