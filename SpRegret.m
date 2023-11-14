function [controllers] = SpRegret(sls,opt,Sreal,Shat,verbose)
%   The function performs the following steps:
%       1. Computes the binary matrices Vx and Vx_hat for the real sparsity matrix and its QI superset.
%       2. Synthesizes the following controllers as described in the paper work:
%          - Centralized controller
%          - Oracle controller
%          - H2 controller
%          - Hinf controller
%          - Regret controller with QI benchmark
%          - Regret controller with centralized benchmark
%   Input arguments:
%       - sls: Struct containing system information.
%       - opt: Struct containing optimization settings.
%       - Sreal: Real system Sparsity Matrix.
%       - Shat: Nearest QI superset.
%       - verbose: Binary flag for verbose output (1/0).
%
%   Output:
%       - controllers: Struct containing synthesized control policies.

%   Example usage:
%       opt.verbose = true;
%       controllers = SpRegret(sls, opt, Sreal, Shat, 1);
%
%   See also FIND_SI, CALCULUS_CENTRALIZED, CALCULUS_BENCHMARK, CALCULUS_REGRET.

% Author: Daniele Martinelli [daniele.martinelli@epfl.ch]
        


sls.Sreal = Sreal;
sls.Shat = Shat;
[sls.Vu_Shat, sls.Vx_Shat] = find_SI(sls,sls.Shat);
[sls.Vu_Sreal, sls.Vx_Sreal] = find_SI(sls,sls.Sreal);


%% Find Controllers
t_start = tic;
tol= 1.e-5;

%% Centralized

fprintf("Starting Centralized --> ")

[Phi_x_centr, Phi_u_centr, obj_h2_centr, obj_hinf_centr] = calculus_centralized(sls, opt,'fro',verbose);

K_centr = Phi_u_centr/(Phi_x_centr);

K_centr = trim(K_centr,tol);

centr.x = Phi_x_centr;
centr.u = Phi_u_centr;
centr.k = K_centr;
fprintf("Finished Centralized \n")
disp("-----------------------------")
%% Oracle
fprintf("Starting Oracle --> ")
is_oracle = true;
[Phi_x_oracle, Phi_u_oracle, obj_h2_oracle, obj_hinf_oracle] = calculus_benchmark(sls, opt, is_oracle,'fro',verbose);

K_oracle = Phi_u_oracle/(Phi_x_oracle);

K_oracle = trim(K_oracle,tol);

oracle.x = Phi_x_oracle;
oracle.u = Phi_u_oracle;
oracle.k = K_oracle;
fprintf("Finished Oracle \n")
disp("-----------------------------")
%% H2
fprintf("Starting H2 --> ")
is_oracle = false;
[Phi_x_h2, Phi_u_h2, obj_h2_h2, obj_hinf_h2] = calculus_benchmark(sls, opt, is_oracle,'fro',verbose);

K_h2 = Phi_u_h2/(Phi_x_h2);
K_h2 = trim(K_h2,tol) ;

h2.x = Phi_x_h2;
h2.u = Phi_u_h2;
h2.k = K_h2;
fprintf("Finished H2 \n")
disp("-----------------------------")
%% Hinf
fprintf("Starting Hinf --> ")
[Phi_x_hinf, Phi_u_hinf, obj_h2_hinf, obj_hinf_hinf] = calculus_benchmark(sls, opt,is_oracle, 'inf',verbose);

K_hinf = Phi_u_hinf/(Phi_x_hinf);
K_hinf = trim(K_hinf,tol);

hinf.x = Phi_x_hinf;
hinf.u = Phi_u_hinf;
hinf.k = K_hinf;
fprintf("Finished Hinf \n")
disp("-----------------------------")
%% Regret Controller
fprintf("Starting Regret QI --> ")
[Phi_x_regret, Phi_u_regret, objective_regret,obj_h2_regret,obj_hinf_regret] = calculus_regret(sls, opt, oracle,verbose);

K_regret = Phi_u_regret/(Phi_x_regret);

K_regret= trim(K_regret,tol);

regret.x = Phi_x_regret;
regret.u = Phi_u_regret;
regret.k = K_regret;
fprintf("Finished Regret QI \n")
disp("-----------------------------")
%% Regret Controller with CENTR benchmark
fprintf("Starting Regret Centralized --> ")
[Phi_x_regret_centr, Phi_u_regret_centr, objective_regret_centr,obj_h2_regret_centr,obj_hinf_regret_centr] = calculus_regret(sls, opt, centr,verbose);

K_regret_centr = Phi_u_regret/(Phi_x_regret);
K_regret_centr= trim(K_regret_centr,tol);

regret_centr.x = Phi_x_regret_centr;
regret_centr.u = Phi_u_regret_centr;
regret_centr.k = K_regret_centr;
fprintf("Finished Regret Centralized \n")
disp("-----------------------------")
%% Finished!
Tend = toc(t_start);
disp("Required time : "+num2str(Tend) + "s")
%% Returning synthesized control Policies
controllers.oracle = oracle;
controllers.h2 = h2;
controllers.hinf = hinf;
controllers.regret = regret;
controllers.centr = centr;
controllers.regret_centr = regret_centr;
end