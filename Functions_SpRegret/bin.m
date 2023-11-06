function [Abin] = bin(A, tol)
if nargin<2
    tol = 1e-5;
end
    indices_zeros = find(abs(A)<tol);
    A(indices_zeros) = 0;
    Abin = A ~= 0;
end

