function [F, G, H, I] = mass_spring_damper_system(n_agents, T_s, vals, method)
% MASS_SPRING_DAMPER_SYSTEM - Generates a discrete-time state-space representation
%                             for a multi-agent mass-spring-damper system.
%
%   This function generates a discrete-time state-space representation for a
%   multi-agent mass-spring-damper system. The system is modeled by a set of
%   interconnected masses, springs, and dampers. The resulting state-space matrices
%   (F, G, H, I) define the system dynamics.
%
%   Input arguments:
%       - n_agents: Number of agents (masses) in the system.
%       - T_s: Sampling time for discretization.
%       - vals: Struct containing system parameters (K, C, Fr, M).
%       - method: (Optional) Discretization method (default: 'zoh').
%
%   Output:
%       - F: State-transition matrix.
%       - G: Input matrix.
%       - H: Output matrix.
%       - I: Feedforward matrix.
%
%   The function constructs the state-space matrices (F, G, H, I) based on the
%   parameters provided in the 'vals' struct. The system dynamics are discretized
%   using the specified method (default: 'zoh') with the given sampling time T_s.
%
%   See also SS, C2D.

    if nargin < 4
        method = 'zoh';
    end
    
    A = zeros(n_agents*2, n_agents*2);

    % First Car
    A(1:2, 1:2) = [0, 1; -(vals.K(1)/vals.M(1)), -(vals.Fr(1) + vals.C(1))/vals.M(1)]; % first block
    A(1:2, 3:4) = [0, 0; (vals.K(1)/vals.M(1)), (vals.C(1)/vals.M(1))]; % next row block

    for i = 2:n_agents
        if i ~= n_agents
            A(2*i-1:2*(i),2*(i-1)-1:2*(i-1)) = [0, 0; (vals.K(i-1)/vals.M(i)), (vals.C(i-1)/vals.M(i))]; % previous block
            A(2*i-1:2*(i),2*(i-1)+1:2*(i)) = [0, 1; -(vals.K(i-1) + vals.K(i))/vals.M(i), -(vals.Fr(i) + vals.C(i-1) + vals.C(i))/vals.M(i)]; % center block
            A(2*i-1:2*(i),2*(i-1)+3:2*(i-1)+4) = [0, 0; (vals.K(i)/vals.M(i)), (vals.C(i)/vals.M(i))]; % next block
        else
            % Leader/Last car
            A(2*i-1:2*(i),2*(i-1)-1:2*(i-1)) = [0, 0; (vals.K(i-1)/vals.M(i)), (vals.C(i-1)/vals.M(i))]; % previous row block
            A(2*i-1:2*(i),2*(i-1)+1:2*(i)) = [0, 1; -(vals.K(i-1)/vals.M(i)), -(vals.Fr(i) + vals.C(i-1))/vals.M(i)]; % final block
        end
    end

    B = zeros(2*n_agents, n_agents);
    for i = 1:n_agents
        B(2*i-1:2*i, i) = [0; 1/vals.M(i)];
    end

    C = eye(2*n_agents);
    D = zeros(2*n_agents, n_agents);

    system_CT = ss(A, B, C, D);
    system_DT = c2d(system_CT, T_s, method);
    F = system_DT.A;
    G = system_DT.B;
    H = system_DT.C;
    I = system_DT.D;
end
