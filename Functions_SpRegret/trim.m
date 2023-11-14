function [A_trim] = trim(A, tol)
    % TRIM - Removes small entries from a matrix, setting them to zero.
%
%   A_trim = trim(A, tol)
%
%   This function removes small entries (values below a specified tolerance) from a matrix A,
%   setting them to zero. The resulting trimmed matrix is returned as A_trim.
%
%   Input arguments:
%       - A: Input matrix to be trimmed.
%       - tol: (Optional) Tolerance value for considering entries as small. Default is 1e-5.
%
%   Output:
%       - A_trim: Matrix A after removing small entries below the specified tolerance.
%
%   The function is useful for cleaning up matrices by setting very small values to zero,
%   improving numerical stability and precision.
%
%   See also ABS.

if nargin<2
    tol = 1e-5;
end
    indices_zeros = find(abs(A)<tol);
    A(indices_zeros) = 0;
    A_trim = A;
end

