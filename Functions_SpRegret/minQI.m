function Smin = minQI(obj, S_0, iterations)
    % MINQI - Finds the minimal QI superset for a given set S_0.
%
%   Smin = minQI(obj, S_0, iterations)

%   The superset is determined based on the system properties stored in the 'obj' object. 
%   It should **NOT BE USED** by itself, but as a private function of CLOSEST_QI
%
%   Input arguments:
%       - obj: Object containing system information (e.g., matrices I, Z, A, B).
%       - S_0: Binary matrix representing the initial set.
%       - iterations: (Optional) Maximum number of iterations for finding the closest QI superset.
%                     Default value is -1 (unlimited iterations).
%
%   Output:
%       - Smin: Binary matrix representing the minimal QI superset for the given S_0.
%
%   See also CLOSEST_QI.

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