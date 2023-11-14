function [Phi_x, Phi_u, obj_h2, obj_hinf] = calculus_benchmark(sls, opt,is_oracle, norm_type, verbose)
% CALCULUS_BENCHMARK - Synthesizes benchmark control policies H2, Hinf, Oracles, using optimization techniques.
%
%   This function synthesizes benchmark control policies as reported in the paper work.
%   It generates state and input matrices (Phi_x and Phi_u) and evaluates
%   the performance in terms of H2 and Hinf norms.
%
%   Input arguments:
%       - sls: Struct containing system information.
%       - opt: Struct containing optimization settings.
%       - is_oracle: Boolean indicating whether to use the oracle sparsity structure or real system's one.
%       - norm_type: String specifying the norm type ('fro' for Frobenius, 'inf' for Hinf).
%       - verbose: Boolean flag for verbose output during optimization.
%
%   Output:
%       - Phi_x: State matrix for the synthesized controller.
%       - Phi_u: Input matrix for the synthesized controller.
%       - obj_h2: H2 norm performance of the synthesized controller.
%       - obj_hinf: Hinf norm performance of the synthesized controller.
%
%   See also SDPVAR, SDPSETTINGS, YALMIPERROR.


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
    % Define the objective function
    if(strcmp(norm_type,'fro'))
        objective = norm(chol(opt.C)*Phi, 'fro');
    else
        objective = norm(chol(opt.C)*Phi, 2);
    end
    constraints = [];

    if is_oracle
        constraints = [constraints, (Phi_u_tilda).*(~(sls.Vu_Shat)) == 0];
        constraints = [constraints, (Phi_x_tilda).*(~(sls.Vx_Shat)) == 0];
    else
        constraints = [constraints, (Phi_u_tilda).*(~(sls.Vu_Sreal)) == 0];
        constraints = [constraints, (Phi_x_tilda).*(~(sls.Vx_Sreal)) == 0];
    end


    % Solve the optimization problem
    options = sdpsettings('verbose', verbose, 'solver', 'mosek');
    sol = optimize(constraints, objective, options);
    if ~(sol.problem == 0)
        error("Error Benchmark calculus (Oracle: "+num2str(is_oracle)+". norm: '"+norm_type+ "'). Problem status: " +num2str(sol.problem) + " ( " + yalmiperror(sol.problem) + ")");
    end
    
    % Extract the closed-loop responses corresponding to the unconstrained clairvoyant controller
    Phi_x = value(Phi_x); 
    Phi_u = value(Phi_u);
    obj_hinf = norm(chol(opt.C)*[Phi_x; Phi_u], 2)^2;
    obj_h2 = norm(chol(opt.C)*[Phi_x; Phi_u], 'fro')^2;
end