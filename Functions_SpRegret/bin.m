function [Abin] = bin(A, tol)
%Function to retrieve the binary version of the input matrix A. The elements of the matrix A with absolute value < tol are put to zero.
if nargin<2
    tol = 1e-5;
end
    indices_zeros = abs(A)<tol;
    A(indices_zeros) = 0;
    Abin = A ~= 0;
end

