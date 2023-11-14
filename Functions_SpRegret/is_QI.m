function [result] = is_QI(S,G)
% IS_QI - Checks if a binary matrix S satisfies the Quantum-Inspired (QI) condition.
%
%   This function checks whether a given binary matrix S satisfies the Quadratic Invariance
%   condition with respect to a given matrix G.
%
%   Input arguments:
%       - S: Sparsity Matrix.
%       - G: Matrix used to define the QI condition.
%
%   Output:
%       - result: Boolean indicating whether S satisfies the QI condition (true) or not (false).
%   See also RANDI.
result = true;
[ny,nu] = size(G);
G_bin = bin(G);
i = 1;
while(i<=ny && result)
    l = 1;
    while(l<=ny && result)
        j = 1;
        while(j<=nu && result)
            k = 1;
            while(k <= nu && result)
                check = S(k,i)*G_bin(i,j)*S(j,l)*(1-S(k,l));
                if (check == 1)
                    %QI condition is not satisfied!!
                    result = false;
                end
                k = k+1;
            end
            j = j + 1;
        end
        l = l + 1;
    end
    i = i+1;

end


