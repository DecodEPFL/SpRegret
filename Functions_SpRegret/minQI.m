function Smin = minQI(obj, S_0, iterations)
    if nargin<3
        iterations = -1;
    else
        if iterations<=0 
            error("Number of max. iterations for finding the closest QI superset is <=0!")
        end
    end
    P12 = (obj.I - obj.Z * obj.A)\ obj.Z * obj.B;
    P12_bin = bin(P12);
    % S_0 = kron(tril(ones(T, T)), sys_Sreal);
    Smin = closest_QI(S_0, P12_bin,iterations);
end