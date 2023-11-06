function Z_star = closest_QI(S,G,iterations)
    %CLOSEST_QI Summary of this function goes here
    %   Detailed explanation goes here

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

