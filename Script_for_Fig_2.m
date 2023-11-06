%% Numerical experiments: comparison between time-averaged incurred control cost
clc;close all; clear;
addpath('./Functions_SpRegret') % Add path to the folder with auxiliary functions
addpath('./Synthesized_Controllers') % Add path to the folder with synthesized controllers
rng(1)

vector_cases = 3:10;
number_of_cases = size(vector_cases,2);
n_max_counters = 1e3;
n_max_confidence = 1e2;
h2_results=zeros(n_max_confidence,number_of_cases);
hinf_results=zeros(n_max_confidence,number_of_cases);
reg_QI_results = zeros(n_max_confidence,number_of_cases);
reg_centr_results =zeros(n_max_confidence,number_of_cases);
T = 30;
N_tf = 20;
lower_bound_unif = -0.5;
upper_bound_unif = 1;
for experiment = 1:number_of_cases

    n_masses = vector_cases(experiment);
    load("Results_n_masses_"+num2str(n_masses) + "_T_"+num2str(T)+"_N_tf_"+num2str(N_tf)+".mat");
    controllers = controllers_S_vs_S_hat;

    h2 = controllers.h2;
    hinf = controllers.hinf;
    oracle = controllers.oracle;
    regret_QI = controllers.regret;



    x0_val = zeros(sls.n,1);

    time_vector_w = T_s*(0:sls.T-2);

    max_number_hittable_masses =n_masses;
    disp("Number of masses: " + num2str(max_number_hittable_masses));
    for n_confidence = 1:n_max_confidence
        n_counter_reg_better = 0;
        n_counter_h2_better = 0;
        n_counter_hinf_better = 0;

        parfor count = 1:n_max_counters
            w = lower_bound_unif+(upper_bound_unif-lower_bound_unif)*rand(sls.n*(sls.T-1),1);
            w = w / norm(w);
            w = [x0_val; w];
            cost_h2_try   = evaluate_policy(opt, h2, w);
            cost_hinf_try = evaluate_policy(opt, hinf, w);
            % Simulate the closed-loop system with the regret-optimal causal controllers
            cost_regret_QI_try = evaluate_policy(opt, regret_QI, w);

            if((cost_h2_try > cost_regret_QI_try) && cost_hinf_try > cost_regret_QI_try)
                n_counter_reg_better = n_counter_reg_better + 1;
            end
            if( cost_regret_QI_try > cost_h2_try && cost_hinf_try > cost_h2_try )
                n_counter_h2_better = n_counter_h2_better + 1;
            end
            if(cost_regret_QI_try > cost_hinf_try && cost_h2_try > cost_hinf_try)
                n_counter_hinf_better = n_counter_hinf_better + 1;
            end



        end
        h2_results(n_confidence,experiment)=n_counter_h2_better;
        hinf_results(n_confidence,experiment)= n_counter_hinf_better;
        reg_QI_results(n_confidence,experiment) = n_counter_reg_better;
    end
end

%% Calculus Confidence Intervals

h2_results=h2_results/n_max_counters*100;
hinf_results=hinf_results/n_max_counters*100;
reg_QI_results = reg_QI_results/n_max_counters*100;
% reg_centr_results =reg_centr_results/n_max_counters*100;


h2_mean = mean(h2_results,1);
hinf_mean=mean(hinf_results,1);
reg_QI_mean = mean(reg_QI_results,1);
% reg_centr_mean= mean(reg_centr_results,1);


h2_std = std(h2_results,1);
hinf_std=std(hinf_results,1);
reg_QI_std = std(reg_QI_results,1);
% reg_centr_std= std(reg_centr_results,1);

confidence = 2;
h2_low = h2_mean-h2_std*confidence;
h2_high = h2_mean+h2_std*confidence;
hinf_low = hinf_mean-hinf_std*confidence;
hinf_high = hinf_mean+hinf_std*confidence;
reg_QI_low = reg_QI_mean-reg_QI_std*confidence;
reg_QI_high = reg_QI_mean+reg_QI_std*confidence;
% reg_centr_low = reg_centr_mean-reg_centr_std*confidence;
% reg_centr_high = reg_centr_mean+reg_centr_std*confidence;

%% Plotting
x_region = [vector_cases vector_cases(end:-1:1)];

color_h2 = [0.00,0.45,0.74];
color_hinf = [0.85,0.33,0.10];
color_reg_QI = [0.49,0.18,0.56];

%Plotting the means
fig = figure;
hold on;
plot(vector_cases,h2_mean,'--o','Color',color_h2,'LineWidth',0.5,'MarkerFaceColor',color_h2);
plot(vector_cases,hinf_mean,'--o','Color',color_hinf,'LineWidth',0.5,'MarkerFaceColor',color_hinf);
plot(vector_cases,reg_QI_mean,'--o','Color',color_reg_QI,'LineWidth',0.5,'MarkerFaceColor',color_reg_QI);


%H2
p = fill(x_region, [h2_low h2_high(end:-1:1)],color_h2);
p.EdgeColor = 'none';
p.FaceAlpha = 0.2;
%Hinf
p = fill(x_region, [hinf_low hinf_high(end:-1:1)],color_hinf);
p.EdgeColor = 'none';
p.FaceAlpha = 0.2;
%Reg_QI
p = fill(x_region, [reg_QI_low reg_QI_high(end:-1:1)],color_reg_QI);
p.EdgeColor = 'none';
p.FaceAlpha = 0.2;

hl = legend({'$\mathbf{K}_{\mathcal{H}_2}$' ;'$\mathbf{K}_{\mathcal{H}_\infty}$' ;'$\mathbf{K}_{R_{QI}}$' ; ''; '';''},...
    'Interpreter','latex','FontSize',14,'Location','west');
ylim([0,100]);
xlabel("Total Number of Masses of the System",'FontSize',12);
ylabel("Best results (%)",'FontSize',12);
grid on; grid minor;

fig.Position = [744,537,491,214];

% exportgraphics(fig, 'Fig_2.pdf', 'ContentType', 'vector');
hold off;