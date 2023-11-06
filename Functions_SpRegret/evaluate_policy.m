function [cum_cost] = evaluate_policy(opt, Phi, w)
%EVALUATE_POLICY computes the cumulative cost incurred applying the policy 
%corresponding to the closed-loop responses in Phi in response to 
%the disturbance realization 

        cum_cost = w'*[Phi.x;Phi.u]'*opt.C*[Phi.x;Phi.u]*w;
end