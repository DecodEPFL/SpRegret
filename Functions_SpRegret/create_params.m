function parameters = create_params(n_masses, K, C, fr, mass)
% CREATE_PARAMS - Creates a struct of system parameters for a multi-mass system.
%
%   This function generates a struct containing system parameters for a multi-mass system,
%   including spring constants, damping coefficients, friction values, and masses. Default
%   values are used if specific parameters are not provided.
%
%   Input arguments:
%       - n_masses: Number of masses in the system.
%       - K: Spring constant (default: 2.0).
%       - C: Damping coefficient (default: 0.2).
%       - fr: Friction value (default: 3.0).
%       - mass: Mass value (default: 2.0).
%
%   Output:
%       - parameters: Struct containing system parameters (K, C, Fr, M).
%
%   The function allows customization of the system parameters by providing specific values
%   for spring constant (K), damping coefficient (C), friction value (fr), and mass (mass).
%   If not specified, default values are used with optional random variation (controlled by std).

%
%   See also RANDN.

% Author: Daniele Martinelli [daniele.martinelli@epfl.ch]

    if nargin < 2
        K = 2.0;
    end
    if nargin < 3
        C = 0.2;
    end
    if nargin < 4
        fr = 3.0;
    end
    if nargin < 5
        mass = 2.0;
    end

    std = 0;%1/20;
    springs_vector = K * ones(n_masses-1, 1) + K * std * randn(n_masses-1, 1);

    dampers_vector = C * ones(n_masses-1, 1) + C * std * randn(n_masses-1, 1);

    frictions_vector = fr * ones(n_masses, 1) + fr * std * randn(n_masses, 1);

    masses_vector = mass * ones(n_masses, 1) + mass * std * randn(n_masses, 1);

    parameters = struct();
    parameters.K = springs_vector;
    parameters.C = dampers_vector;
    parameters.Fr = frictions_vector;
    parameters.M = masses_vector;
end
