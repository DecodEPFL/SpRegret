function [w, obj] = calc_w_max_Delta(sls,Delta)
    [evectors_Delta, evalues_Delta] = eig(Delta, 'vector');
    [~, max_index] = max(evalues_Delta);
    delta = evectors_Delta(:, max_index);
    obj = delta' * Delta * delta;
    w = delta;
end


