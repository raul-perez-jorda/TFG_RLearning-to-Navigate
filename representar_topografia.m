clear all; close all;

% load('LearningData_vw/datos_topograficos.mat', 'v_samples', 'w_samples');
load('LearningData_T/datos_topograficos.mat', 'TL_samples', 'TG_samples');


movmean_const = 200;
grid_const = 100;

final_rewards = [];
final_durations = [];
final_distancias_siguiendo_pared = [];

% carpeta_LearningData = "LearningData_vw";
% carpeta_LearningData = "LearningData_T_NoNormalizado"
carpeta_LearningData = "LearningData_T";

% versiones = ["6D_IP_1"; "6D_IP_2"; "6D_IP_3"; "6D_IP_4"; "6D_IP_5"; "6D_IP_6"; "6D_IP_7"; "6D_IP_8"];
% versiones = ["6D_IP_T1"; "6D_IP_T2"; "6D_IP_T3"; "6D_IP_T4"; "6D_IP_T5"; "6D_IP_T6"; "6D_IP_T7"; "6D_IP_T8"]
versiones = ["6D_IP_T1"; "6D_IP_T2"; "6D_IP_T3"; "6D_IP_T4"; "6D_IP_T5"; "6D_IP_T6"; "6D_IP_T7"; "6D_IP_T8"; "6D_IP_T9"; "6D_IP_T10"; ...
    "6D_IP_T11"; "6D_IP_T12"; "6D_IP_T13"; "6D_IP_T14"; "6D_IP_T17"]%; "6D_IP_T16"; "6D_IP_T15"; "6D_IP_T18"; "6D_IP_T19"; "6D_IP_T20"; ...
    % "6D_IP_T21"; "6D_IP_T22"; "6D_IP_T23"; "6D_IP_T24"; "6D_IP_T25"; "6D_IP_T26"; "6D_IP_T27"; "6D_IP_T28"];
% versiones = ["6D_IP_T9"; "6D_IP_T10"; "6D_IP_T11"; "6D_IP_T12"; "6D_IP_T19"; "6D_IP_T20"; "6D_IP_T21"; "6D_IP_T22"];
% x = [TL_samples(1:12), TL_samples(19:22)];
% y = [TG_samples(1:12), TG_samples(19:22)];
x = TL_samples(1:length(versiones));
y = TG_samples(1:length(versiones));

for i = 1:1:length(versiones)
    load(carpeta_LearningData+'/vO'+versiones(i)+'/Qlearning_data_vO'+versiones(i)+'_mas_reciente.mat','total_reward_per_episode', 'total_duration_per_episode', 'distancias_siguiendo_pared');
    % load(carpeta_LearningData+'/vO'+versiones(i)+'/Qlearning_data_vO'+versiones(i)+'_mas_reciente.mat','total_reward_per_episode', 'total_duration_per_episode');
    reward_per_episode_soft = movmean(total_reward_per_episode, movmean_const);
    duration_per_episode_soft = movmean(total_duration_per_episode, movmean_const);
    siguiendopared_per_episode_soft = movmean(distancias_siguiendo_pared, movmean_const);

    final_rewards = [final_rewards, reward_per_episode_soft(end)];
    final_durations = [final_durations, duration_per_episode_soft(end)];
    final_distancias_siguiendo_pared = [final_distancias_siguiendo_pared, siguiendopared_per_episode_soft(end)];
end

% x = v_samples;
% y = w_samples;

plot_funcion_multivariable(x,y,final_rewards,grid_const,"Recompensa",1);
plot_funcion_multivariable(x,y,final_durations,grid_const,"Tiempo sobrevivido",2);
plot_funcion_multivariable(x,y,final_distancias_siguiendo_pared,grid_const,"Distancia recorrida siguiendo pared",3);

function plot_funcion_multivariable(x,y,z,grid_const,label,num_figure)
    figure(num_figure)
    xlin = linspace(min(x), max(x), grid_const);
    ylin = linspace(min(y), max(y), grid_const);
    [X,Y] = meshgrid(xlin, ylin);
    % Z = griddata(x,y,z,X,Y,'natural');
    % Z = griddata(x,y,z,X,Y,'cubic');
    Z = griddata(x,y,z,X,Y,'v4');
    mesh(X,Y,Z)
    axis tight; hold on
    plot3(x,y,z,'.','MarkerSize',15)
    xlabel('TL (s)');
    ylabel('TG (s)');
    zlabel(label);
end



