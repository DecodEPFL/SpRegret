function [Vu,Vx] = find_SI(sls,S,init_Vx)
% FIND_SI - Finds structural identifiability matrices for a given system.
%   This function identifies the Vx for a given system. 
%   The algorithm has been first introduced in Furieri et al. "Sparsity invariance for convex design of distributed controllers." 
%   IEEE Transactions on Control of Network Systems 7.4 (2020): 1836-1847.
%
%   Input arguments:
%       - sls: Struct containing system information (e.g., number of states, inputs, and time steps).
%       - S: Binary matrix representing the system's structure.
%       - init_Vx: (Optional) Initial structural identifiability matrix for state variables.
%
%   Output:
%       - Vu
%       - Vx
%
%   See also RANDI.


Vu = S;
if nargin<3
    R_star = ones(sls.n*sls.T, sls.n*sls.T);
else
    R_star = init_Vx;
end
    for i = 1:(sls.m*sls.T)
        for k = 1:(sls.n*sls.T)
            
            if Vu(i, k) == 0
                for j = 1:sls.n*sls.T
                    if Vu(i, j) == 1
                        R_star(j, k) = 0;
                    end
                end
            end
        end
    end

    Vx = R_star;
end

