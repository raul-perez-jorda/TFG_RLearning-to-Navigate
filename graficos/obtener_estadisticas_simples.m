clear; close all;

% carpeta_LearningData = "LearningData";
% carpeta_LearningData = "LearningData_bigEyes";
% carpeta_LearningData = "LearningData_Hiperparametros";
% carpeta_LearningData = "LearningData_vw";
% carpeta_LearningData = "LearningData9_numLasers";
% carpeta_LearningData = "LearningData10_numLasersMinDistRegion";

carpeta_LearningData = "LearningData12_searchingGoodPerformance";
versiones = ["1", "2", "3", "4"];


% versiones = ["3"; "3D"; "4"; "4D"; "5"; "5D"; "6"; "6D"; "7"; "7D"; "8"; "8D"];
% versiones = ["3"; "4"; "5"; "6"; "7"; "8"];
% versiones = ["3D"; "4D"; "5D"; "6D"; "7D"; "8D"];
% versiones = ["6"; "6D"; "7"; "7D"; "8"; "8D"];
% versiones = ["6D"; "6D_IT"; "6D_IC"; "6D_TP"; "6D_TT"; "6D_TC"; "6D_CP"; "6D_CT"; "6D_CC"];
% versiones = ["6"; "4D"];
% versiones = ["6D_IP_T1"; "6D_IP_T2"; "6D_IP_T3"; "6D_IP_T4"; "6D_IP_T5"; "6D_IP_T6"; "6D_IP_T7"; "6D_IP_T8"];
% versiones = ["6D_IP_T1"; "6D_IP_T2"; "6D_IP_T3"; "6D_IP_T4"; "6D_IP_T5"; "6D_IP_T6"; "6D_IP_T7"; "6D_IP_T8"; "6D_IP_T9"; "6D_IP_T10"; "6D_IP_T11"; "6D_IP_T12"; "6D_IP_T13"; "6D_IP_T14"; "6D_IP_T15"];
% versiones = ["6D_IP_T9"; "6D_IP_T10"; "6D_IP_T11"; "6D_IP_T12"; "6D_IP_T19"; "6D_IP_T20"; "6D_IP_T21"; "6D_IP_T22"];


figure(1)
%clf
hold on

for i = 1:1:length(versiones)
    load('LearningData_Folders/'+carpeta_LearningData+'/vO'+versiones(i)+'/Qlearning_data_vO'+versiones(i)+'_mas_reciente.mat','total_reward_per_episode', 'total_duration_per_episode', 'Visitas', 'distancias_siguiendo_pared'); 
    % movmean_const = ceil(i * 0.5); % el 20% del numero de episodios, se va haciendo mayor en función del número de episodios
    movmean_const = 100;

    subplot(3,1,1)
    plot(movmean(total_reward_per_episode,movmean_const))
    hold on
    subplot(3,1,2)
    plot(movmean(total_duration_per_episode,movmean_const))
    hold on
    subplot(3,1,3)
    plot(movmean(distancias_siguiendo_pared,movmean_const))
    hold on
end

subplot(3,1,1)
legend(versiones)
title('Recompensa obtenida por episodio (media móvil)')
xlabel('Episodio')
ylabel('Recompensa')

subplot(3,1,2)
legend(versiones)
title('Tiempo sobrevivido obtenida por episodio (media móvil)')
xlabel('Episodio')
ylabel('Tiempo (s)')

subplot(3,1,3)
legend(versiones)
title('Distancia siguiendo la pared (media móvil)')
xlabel('Episodio')
ylabel('Distancia (m)')


hold off




