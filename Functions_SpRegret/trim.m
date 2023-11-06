function [A_trim] = trim(A, tol)
if nargin<2
    tol = 1e-5;
end
    indices_zeros = find(abs(A)<tol);
    A(indices_zeros) = 0;
    A_trim = A;
end

