clear all; close all; clc;
rng(123)


A = 0.5;
B = 1;

sys.n = size(A, 1);   % Order of the system: state dimension
sys.m = size(B, 2);


T = 3;
sls.A = kron(eye(T), A);
sls.B = kron(eye(T), B);
sls.I = eye(sys.n*T);
sls.Z = [zeros(sys.n, sys.n*(T-1)) zeros(sys.n, sys.n); eye(sys.n*(T-1)) zeros(sys.n*(T-1), sys.n)];
opt.C = blkdiag(eye(T*sys.n), eye(T*sys.m));


% K = diag(randn(3,1));
% K = round(K,2);
K = -eye(3);
Phi_x = inv(sls.I - sls.Z*(sls.A + sls.B*K));
Phi_u = K/(sls.I - sls.Z*(sls.A + sls.B*K)) ;
Phi = [Phi_x;Phi_u];

% Searching for K_hat
for i=1:1e6
    K_hat = randn(sys.m*T,sys.n*T);
    K_hat = round(K_hat,1);   %Nicer result!
    K_hat = tril(K_hat);
    % K_hat(2,1) = 0; %Sparsity1
    Phi_x_hat = inv(sls.I - sls.Z*(sls.A + sls.B*K_hat));
    Phi_u_hat = K_hat/(sls.I - sls.Z*(sls.A + sls.B*K_hat)) ;
    Phi_hat = [Phi_x_hat;Phi_u_hat];
    matrix = Phi_hat'*opt.C*Phi_hat - Phi'*opt.C*Phi;
    lowest_eig = min(eig(matrix));
    if (all(eig(matrix) >= 0))
        disp("FOUND a solution! Iteration number: " + num2str(i))
        disp("K");
        disp(K);
        disp("K_hat");
        disp(K_hat);
        break;
    end
end


%% Testing
if (exist('K','var') == 1)
    for i=1:1e5
        x0_i = randn();
        w_i = randn(T-1,1);
        delta_i = [x0_i;w_i];
        J_i = delta_i'*Phi'*opt.C*Phi*delta_i;
        J_hat_i = delta_i'*Phi_hat'*opt.C*Phi_hat*delta_i;
        if(J_i > J_hat_i)
            error("What you have discovered is not true... Iteration number" + num2str(i))
        end
    end
    disp("Finished without Error!")
end


