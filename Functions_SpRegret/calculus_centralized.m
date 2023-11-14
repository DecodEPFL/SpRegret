function [Phi_x, Phi_u, obj_h2, obj_hinf] = calculus_centralized(sls, opt, norm_type, verbose)
% CALCULUS_CENTRALIZED - Synthesizes a centralized benchmark control policies
%
%   This function synthesizes benchmark the centralized control policies.
%   It generates state and input matrices (Phi_x and Phi_u) and evaluates
%   the performance in terms of H2 and Hinf norms.
%
%   Input arguments:
%       - sls: Struct containing system information.
%       - opt: Struct containing optimization settings.
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
    % Phi_x = kron(temp, vector_Phi_x(1:sls.n,:));
    for i = 1:(opt.N_tf - 1)
        temp = diag(ones(opt.T-i,1), -i);
        Phi_u_tilda = Phi_u_tilda + kron(temp, vector_Phi_u((i*sls.m + 1):((i + 1)*sls.m),:));
        % Phi_x = Phi_x + kron(temp, vector_Phi_x((i*sls.n + 1):((i + 1)*sls.n),:));
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
   
    % Solve the optimization problem
    options = sdpsettings('verbose', verbose, 'solver', 'mosek');
    sol = optimize(constraints, objective, options);
    if ~(sol.problem == 0)
        error("Something went wrong... Problem status: " +num2str(sol.problem) + " ( " + yalmiperror(sol.problem) + ")");
    end
    
    % Extract the closed-loop responses corresponding to the unconstrained clairvoyant controller
    Phi_x = value(Phi_x); 
    Phi_u = value(Phi_u);

    obj_hinf = norm(chol(opt.C)*[Phi_x; Phi_u], 2)^2;
    obj_h2 = norm(chol(opt.C)*[Phi_x; Phi_u], 'fro')^2;

end