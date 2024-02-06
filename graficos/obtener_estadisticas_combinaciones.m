clear all; close all;

min_episode_representation = 1;
max_episode_representation = 1000;
array_rewards = [];
array_durations = [];

carpeta_LearningData = "LearningData";
% carpeta_LearningData = "LearningData_bigEyes";
% carpeta_LearningData = "LearningData_Hiperparametros";
% carpeta_LearningData = "LearningData_vw";
% carpeta_LearningData = "LearningData_T";

% versiones = ["3"; "3D"; "4"; "4D"; "5"; "5D"; "6"; "6D"; "7"; "7D"; "8"; "8D"];
% versiones = ["3"; "4"; "5"; "6"; "7"; "8"];
% versiones = ["3D"; "4D"; "5D"; "6D"; "7D"; "8D"];
versiones = ["3"; "4"; "5"; "6"; "7"; "8"; "3D"; "4D"; "5D"; "6D"; "7D"; "8D"];
% versiones = ["6"; "6D"; "7"; "7D"; "8"; "8D"];
% versiones = ["6D"; "6D_IT"; "6D_IC"; "6D_TP"; "6D_TT"; "6D_TC"; "6D_CP"; "6D_CT"; "6D_CC"];
% versiones = ["6D"];
% versiones = ["6D_IP_1"; "6D_IP_2"; "6D_IP_3"; "6D_IP_4"; "6D_IP_5"; "6D_IP_6"; "6D_IP_7"; "6D_IP_8"];
% versiones = ["6D_IP_T1"; "6D_IP_T2"; "6D_IP_T3"; "6D_IP_T4"; "6D_IP_T5"; "6D_IP_T6"; "6D_IP_T7"; "6D_IP_T8"; "6D_IP_T9"; "6D_IP_T10"; "6D_IP_T11"; "6D_IP_T12"; "6D_IP_T19"; "6D_IP_T20"; "6D_IP_T21"; "6D_IP_T22"];
% versiones = ["6D_IP_T9"; "6D_IP_T10"; "6D_IP_T11"; "6D_IP_T12"; "6D_IP_T19"; "6D_IP_T20"; "6D_IP_T21"; "6D_IP_T22"];

movmean_const = 200;

figure(1)
%clf
hold on

for i = 1:1:length(versiones)
    hold on
    load(carpeta_LearningData+'/vO'+versiones(i)+'/Qlearning_data_vO'+versiones(i)+'_mas_reciente.mat','total_reward_per_episode', 'total_duration_per_episode', 'Visitas');    

    subplot(2,1,1)
    soft_rewards_episode = movmean(total_reward_per_episode(min_episode_representation:max_episode_representation),movmean_const);
    array_rewards = [array_rewards, soft_rewards_episode];

    hold on
    subplot(2,1,2)
    soft_duration_episode = movmean(total_duration_per_episode(min_episode_representation:max_episode_representation),movmean_const);
    array_durations = [array_durations, soft_duration_episode];
end

rewards_impares = mean(array_rewards(:,[1,3,5,7,9,11]),2);
rewards_pares = mean(array_rewards(:,[2,4,6,8,10,12]),2);
durations_impares = mean(array_durations(:,[1,3,5,7,9,11]),2);
durations_pares = mean(array_durations(:,[2,4,6,8,10,12]),2);

% rewards_trisector = mean(array_rewards(:,[1:6]),2);
% rewards_bisector = mean(array_rewards(:,[7:12]),2);
% durations_trisector = mean(array_durations(:,[1:6]),2);
% durations_bisector = mean(array_durations(:,[7:12]),2);

% rewards_alpha_1 = mean(array_rewards(:,1:3),2);
% rewards_alpha_2 = mean(array_rewards(:,4:6),2);
% rewards_alpha_3 = mean(array_rewards(:,7:9),2);
% durations_alpha_1 = mean(array_durations(:,1:3),2);
% durations_alpha_2 = mean(array_durations(:,4:6),2);
% durations_alpha_3 = mean(array_durations(:,7:9),2);

% rewards_epsilon_1 = mean(array_rewards(:,[1,4,7]),2);
% rewards_epsilon_2 = mean(array_rewards(:,[2,5,8]),2);
% rewards_epsilon_3 = mean(array_rewards(:,[3,6,9]),2);
% durations_epsilon_1 = mean(array_durations(:,[1,4,7]),2);
% durations_epsilon_2 = mean(array_durations(:,[2,5,8]),2);
% durations_epsilon_3 = mean(array_durations(:,[3,6,9]),2);

subplot(2,1,1)
plot(rewards_impares); plot(rewards_pares);
legend('Numero impar de láseres', 'Numero par de láseres');
% plot(rewards_trisector); plot(rewards_bisector);
% legend('Sectores izq.-frontal-der.', 'Sectores frontal-der.')
% plot(rewards_alpha_1); plot(rewards_alpha_2); plot(rewards_alpha_3); 
% legend('Alpha: Descenso infinito', 'Alpha: Tiempo de vida', 'Alpha: Constante')
% plot(rewards_epsilon_1); plot(rewards_epsilon_2); plot(rewards_epsilon_3); 
% legend('Epsilon: Porcentaje no visitado', 'Epsilon: Tiempo de vida', 'Epsilon: Constante')
title('Recompensa obtenida por episodio (media móvil)')
xlabel('Episodio')
ylabel('Recompensa')

subplot(2,1,2)
plot(durations_impares); plot(durations_pares);
legend('Numero impar de láseres', 'Numero par de láseres');
% plot(durations_trisector); plot(durations_bisector);
% legend('Sectores izq.-frontal-der.', 'Sectores frontal-der.')
% plot(durations_epsilon_1); plot(durations_epsilon_2); plot(durations_epsilon_3); 
% legend('Epsilon: Porcentaje no visitado', 'Epsilon: Tiempo de vida', 'Epsilon: Constante')
title('Tiempo sobrevivido por episodio (media móvil)')
xlabel('Episodio')
ylabel('Tiempo (s)')


hold off

