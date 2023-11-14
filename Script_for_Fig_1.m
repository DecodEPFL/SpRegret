%   Script used to generate Figure 1 reported in the paper.
%
%   This script synthesizes and compares different control policies for a dynamic system
%   with variable mass affected by external disturbances. The system has multiple masses,
%   and the objective is to study the performance of controllers under varying degrees
%   of mass disturbance. The script evaluates the performance of H2, Hinf, Regret QI, and
%   Regret Centralized controllers and compares their results.
%
%   Note: Ensure that the required functions and controllers are available in the specified
%   folders ('./Functions_SpRegret' and './Synthesized_Controllers').
%
%   Author: Daniele Martinelli [daniele.martinelli@epfl.ch]
clc;close all; clear;
addpath('./Functions_SpRegret') % Add path to the folder with auxiliary functions
addpath('./Synthesized_Controllers') % Add path to the folder with synthesized controllers
rng(1)




n_masses=10;
T = 30;
N_tf = 20;
lower_bound_unif = -0.5;
upper_bound_unif = 1;
load("Results_n_masses_"+num2str(n_masses) + "_T_"+num2str(T)+"_N_tf_"+num2str(N_tf)+".mat");



controllers = controllers_S_vs_S_hat;


h2 = controllers.h2;
hinf = controllers.hinf;
oracle = controllers.oracle;
regret_QI = controllers.regret;
regret_centr = controllers.regret_centr;

sls.sigma_w = 1;
n_max_counters = 1e3;
n_max_confidence = 1e2;
x0_val = zeros(sls.n,1);

time_vector_w = T_s*(0:sls.T-2);

h2_results=zeros(n_max_confidence,n_masses);
hinf_results=zeros(n_max_confidence,n_masses);
reg_QI_results = zeros(n_max_confidence,n_masses);
reg_centr_results =zeros(n_max_confidence,n_masses);

max_number_hittable_masses_vector =1:n_masses;
for n_confidence = 1:n_max_confidence
    for max_number_hittable_masses = max_number_hittable_masses_vector
        % disp("Max Number of hittable masses: " + num2str(max_number_hittable_masses));

        n_counter_reg_better = 0;
        n_counter_h2_better = 0;
        n_counter_hinf_better = 0;
        n_counter_reg_centr_better =0;
        n = sls.n;
        T = sls.T;
        parfor count = 1:n_max_counters
            rand_matrix = zeros(n,T-1);
            rand_number_hit_masses = (max_number_hittable_masses);
            hit_masses = randi(n_masses,[1,rand_number_hit_masses]);
            hit_masses = unique(hit_masses);
            while( size(hit_masses)< rand_number_hit_masses)
                hit_masses = randi(n_masses,[1,rand_number_hit_masses]);
                hit_masses = unique(hit_masses);
            end
            for car=hit_masses
                rand_matrix(2*car-1:2*car,:) =lower_bound_unif+(upper_bound_unif-lower_bound_unif)*rand(2,T-1);
            end
            rand_matrix = rand_matrix / norm(rand_matrix) ;
            w = [x0_val; reshape(rand_matrix,[n * (T-1), 1])];
            cost_h2_try   = evaluate_policy(opt, h2, w);
            cost_hinf_try = evaluate_policy(opt, hinf, w);
            % Simulate the closed-loop system with the regret-optimal causal controllers
            cost_regret_QI_try = evaluate_policy(opt, regret_QI, w);
            cost_regret_centr_try = evaluate_policy(opt, regret_centr, w);


            if(cost_h2_try > cost_regret_QI_try && cost_hinf_try > cost_regret_QI_try && cost_regret_centr_try > cost_regret_QI_try) %|| (cost_h2_try > cost_regret_centr_try && cost_hinf_try > cost_regret_centr_try)
                n_counter_reg_better = n_counter_reg_better + 1;
            end
            if(cost_regret_QI_try > cost_h2_try && cost_hinf_try > cost_h2_try && cost_regret_centr_try > cost_h2_try )
                n_counter_h2_better = n_counter_h2_better + 1;
            end
            if(cost_regret_QI_try > cost_hinf_try && cost_h2_try > cost_hinf_try && cost_regret_centr_try > cost_hinf_try)
                n_counter_hinf_better = n_counter_hinf_better + 1;
            end

            if(cost_regret_QI_try > cost_regret_centr_try && cost_h2_try > cost_regret_centr_try && cost_hinf_try > cost_regret_centr_try)
                n_counter_reg_centr_better = n_counter_reg_centr_better + 1;
            end

        end
        h2_results(n_confidence,max_number_hittable_masses)=n_counter_h2_better;
        hinf_results(n_confidence,max_number_hittable_masses)= n_counter_hinf_better;
        reg_QI_results(n_confidence,max_number_hittable_masses) = n_counter_reg_better;
        reg_centr_results(n_confidence,max_number_hittable_masses) =n_counter_reg_centr_better;

    end
end

%% Calculus Confidence Interval
max_perc_affected_masses = max_number_hittable_masses_vector/n_masses*100;

h2_results=h2_results/n_max_counters*100;
hinf_results=hinf_results/n_max_counters*100;
reg_QI_results = reg_QI_results/n_max_counters*100;
reg_centr_results =reg_centr_results/n_max_counters*100;


h2_mean = mean(h2_results,1);
hinf_mean=mean(hinf_results,1);
reg_QI_mean = mean(reg_QI_results,1);
reg_centr_mean= mean(reg_centr_results,1);


h2_std = std(h2_results,1);
hinf_std=std(hinf_results,1);
reg_QI_std = std(reg_QI_results,1);
reg_centr_std= std(reg_centr_results,1);

confidence = 2;
h2_low = h2_mean-h2_std*confidence;
h2_high = h2_mean+h2_std*confidence;
hinf_low = hinf_mean-hinf_std*confidence;
hinf_high = hinf_mean+hinf_std*confidence;
reg_QI_low = reg_QI_mean-reg_QI_std*confidence;
reg_QI_high = reg_QI_mean+reg_QI_std*confidence;
reg_centr_low = reg_centr_mean-reg_centr_std*confidence;
reg_centr_high = reg_centr_mean+reg_centr_std*confidence;

%% Plotting
x_region = [max_perc_affected_masses max_perc_affected_masses(end:-1:1)];

color_h2 = [0.00,0.45,0.74];
color_hinf = [0.85,0.33,0.10];
color_reg_QI = [0.49,0.18,0.56];
color_reg_centr = [0.93,0.69,0.13];

%Plotting the means
fig = figure;
hold on;
plot(max_perc_affected_masses,h2_mean,'--o','Color',color_h2,'LineWidth',0.5,'MarkerFaceColor',color_h2);
plot(max_perc_affected_masses,hinf_mean,'--o','Color',color_hinf,'LineWidth',0.5,'MarkerFaceColor',color_hinf);
plot(max_perc_affected_masses,reg_QI_mean,'--o','Color',color_reg_QI,'LineWidth',0.5,'MarkerFaceColor',color_reg_QI);
plot(max_perc_affected_masses,reg_centr_mean,'--o','Color',color_reg_centr,'LineWidth',0.5,'MarkerFaceColor',color_reg_centr);
%H2
p = fill(x_region, [h2_low h2_high(end:-1:1)],color_h2);      
p.EdgeColor = 'none';   
p.FaceAlpha = 0.2;
%hinf
p = fill(x_region, [hinf_low hinf_high(end:-1:1)],color_hinf);    
p.EdgeColor = 'none';   
p.FaceAlpha = 0.2;
%reg_QI
p = fill(x_region, [reg_QI_low reg_QI_high(end:-1:1)],color_reg_QI);      
p.EdgeColor = 'none';   
p.FaceAlpha = 0.2;
%reg_centr
p = fill(x_region, [reg_centr_low reg_centr_high(end:-1:1)],color_reg_centr);    
p.EdgeColor = 'none';   
p.FaceAlpha = 0.2;
hl = legend({'$\mathbf{K}_{\mathcal{H}_2}$' ;'$\mathbf{K}_{\mathcal{H}_\infty}$' ;'$\mathbf{K}_{R_{QI}}$' ;...
    '$\mathbf{K}_{R_{C}}$' ;'' ;''; '';''}, 'Interpreter','latex','FontSize',14,'Location','best');
xlim([10,100]);
ylim([0,100]);
xlabel("Max. Percentage of Affected Masses (%)",'FontSize',12);
ylabel("Best results (%)",'FontSize',12);
grid on; grid minor;

fig.Position = [744,537,491,214];
% exportgraphics(fig, 'Fig_1.pdf', 'ContentType', 'vector');
hold off;
