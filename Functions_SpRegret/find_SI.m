function [Vu,Vx] = find_SI(sls,S,init_Vx)
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

