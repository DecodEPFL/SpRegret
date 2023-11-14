function Z_star = closest_QI(S,G,iterations)
% CLOSEST_QI - Finds the closest QI matrix to a given matrix S.
%
%   This function iteratively updates a binary matrix Z_star to find the nearest Quadratically Invariant
%   (QI) superset to a given matrix S.
%
%   Input arguments:
%       - S: The target matrix for which the closest QI matrix is sought.
%       - G: The binary matrix representing the mapping between the input u(t) and the output x(t).
%       - iterations: (Optional) The number of iterations for the update process.
%                     If not specified (-1), the number of iterations is determined automatically.
%
%   Output:
%       - Z_star: The closest QI matrix to the given matrix S.


    G_bin = bin(G);
    [nyT,nuT] = size(G);
    n_loops = min(nyT, nuT);
    n_loops = ceil(log2(n_loops));

    Z_star = bin(S);
    if(iterations~= -1)
        n_loops = min(n_loops,iterations);
    end
    for i = 1:n_loops
        Z_star = bin(Z_star + bin(Z_star * G_bin * Z_star));
    end

end

