function parameters = create_params(n_masses, K, C, fr, mass)
    % Set default values if not provided
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
