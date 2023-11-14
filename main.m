% In this script, you can set the linear system to control and the sparsity matrix S 
% Execute the script to obtain the controllers H2, Hinf, SpRegret with $\mathbf{K}{QI}$, and SpRegret with cenrtalized oracle $\mathbf{K}{C}$.
% The resulting controllers are collected inside a struct named "controllers_S_vs_S_hat" and saved in a .mat file.
clear all; close all; clc;
addpath('./Functions_SpRegret');  % Add path to the folder with auxiliary functions
rng(10);                          % Set random seed for reproducibility


n_masses = 3; %number of masses

T = 30;  %Time Horizon
N_tf = 20; %It is the number of previous time istants used to compute the control input at time t ...
                        %e.g., u(t) = func(x(t), x(t-1), ... x(t-N_tf))
                        %with N_tf <=T


%% Creation SLS + OPT
T_s = 0.5; %Sampling time
k = 0.5;   %Spring Coefficient
c = 0.5;   %Damper coefficient
fr = 0;    %Viscuous Friction
mass =0.1; %mass
vals = create_params(n_masses,k,c,fr,mass); %Parameters of the model. They can be defined differently for each mass.
[A,B,~,~] = mass_spring_damper_system(n_masses,T_s,vals,'zoh');

sys.A = A;
sys.B = B;
sys.n = size(sys.A, 1);   % Order of the system: state dimension
sys.m = size(sys.B, 2);   % Number of input channels

opt.Qt = 1*eye(sys.n);  %Stage cost: state weight matrix
opt.Rt = 10*eye(sys.m); % Stage cost: input weight matrix

opt.T = T; % Control horizon
opt.N_tf = N_tf;    % Past Time Window Control Horizon

opt.Q = kron(eye(opt.T), opt.Qt); % State cost matrix
opt.R = kron(eye(opt.T), opt.Rt); % Input cost matrix
opt.C = blkdiag(opt.Q, opt.R); % Cost matrix


%%
sls.T = opt.T;

sls.A = kron(eye(opt.T), sys.A);
sls.B = kron(eye(opt.T), sys.B);
sls.n = sys.n;
sls.m = sys.m;
sls.T_s = T_s;
sls.I = eye(sys.n*opt.T); % Identity matrix and block-downshift operator
sls.Z = [zeros(sys.n, sys.n*(opt.T-1)) zeros(sys.n, sys.n); eye(sys.n*(opt.T-1)) zeros(sys.n*(opt.T-1), sys.n)];

sls.P12 = (sls.I - sls.Z * sls.A)\ (sls.Z * sls.B); 

%% Evaluating S and S_hat structures:
S = zeros(sys.m, sys.n);  %Sparsity matrix S
for i = 1:sls.m
    S(i, 2*i-1) = 1; %own position
    S(i, 2*i) = 1; %own velocity
    if(i~=sls.m)
        S(i,2*i + 1) = 1; %next mass's position
    end
end
S(:,2*n_masses - 1: 2*n_masses) = 1; %Leader Position

S = kron(tril(ones(opt.T, opt.T)),S);

S_hat = minQI(sls,S);   %QI Oracle Sparsity Matrix


%% Control Synthesis
verbose = 0; %Enable to print the output of optimization steps

try
    [controllers_S_vs_S_hat] = SpRegret(sls,opt,S,S_hat, verbose);
catch ME
    disp("Caught Error! "+ ME.message)
end
disp("-----------------------------")
disp("Finished")

%% Saving

save("Results_n_masses_"+num2str(n_masses) + "_T_"+num2str(T)+"_N_tf_"+num2str(N_tf)+".mat");