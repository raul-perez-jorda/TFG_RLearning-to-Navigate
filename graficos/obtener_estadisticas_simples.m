clear all; close all;

% ver_estados_descubiertos = false;
min_episode_representation = 1;
initial_max_episode_representation = 1000;
tiene_distancia = true; % se almacenaba la distancia siguiendo la pared?

% carpeta_LearningData = "LearningData";
% carpeta_LearningData = "LearningData_bigEyes";
% carpeta_LearningData = "LearningData_Hiperparametros";
% carpeta_LearningData = "LearningData_vw";
carpeta_LearningData = "LearningData_T";

% versiones = ["3"; "3D"; "4"; "4D"; "5"; "5D"; "6"; "6D"; "7"; "7D"; "8"; "8D"];
% versiones = ["3"; "4"; "5"; "6"; "7"; "8"];
% versiones = ["3D"; "4D"; "5D"; "6D"; "7D"; "8D"];
% versiones = ["6"; "6D"; "7"; "7D"; "8"; "8D"];
% versiones = ["6D"; "6D_IT"; "6D_IC"; "6D_TP"; "6D_TT"; "6D_TC"; "6D_CP"; "6D_CT"; "6D_CC"];
% versiones = ["6"; "4D"];
% versiones = ["6D_IP_T1"; "6D_IP_T2"; "6D_IP_T3"; "6D_IP_T4"; "6D_IP_T5"; "6D_IP_T6"; "6D_IP_T7"; "6D_IP_T8"];
versiones = ["6D_IP_T1"; "6D_IP_T2"; "6D_IP_T3"; "6D_IP_T4"; "6D_IP_T5"; "6D_IP_T6"; "6D_IP_T7"; "6D_IP_T8"; "6D_IP_T9"; "6D_IP_T10"; "6D_IP_T11"; "6D_IP_T12"; "6D_IP_T13"; "6D_IP_T14"; "6D_IP_T15"];
% versiones = ["6D_IP_T9"; "6D_IP_T10"; "6D_IP_T11"; "6D_IP_T12"; "6D_IP_T19"; "6D_IP_T20"; "6D_IP_T21"; "6D_IP_T22"];

movmean_const = 200;

figure(1)
%clf
hold on

for i = 1:1:length(versiones)
    load(carpeta_LearningData+'/vO'+versiones(i)+'/Qlearning_data_vO'+versiones(i)+'_mas_reciente.mat','total_reward_per_episode', 'total_duration_per_episode', 'Visitas', 'distancias_siguiendo_pared');

    if(initial_max_episode_representation>length(total_duration_per_episode))
        max_episode_representation = length(total_duration_per_episode);
    else
        max_episode_representation = initial_max_episode_representation;
    end
    
    % if ver_estados_descubiertos
    %     total_pairs_discovered_per_episode = count_discoverings(versiones(i), carpeta_LearningData);
    % 
    %     descubierto_ep_anterior = 0;
    %     relative_pairs_discovered_per_episode = [];
    %     for k = 1:1:length(total_pairs_discovered_per_episode)
    %         relative_pairs_discovered_per_episode(k) = total_pairs_discovered_per_episode(k) - descubierto_ep_anterior;
    %         descubierto_ep_anterior = total_pairs_discovered_per_episode(k);
    %     end
    % 
    %     subplot(3,1,3)
    %     plot(movmean(relative_pairs_discovered_per_episode, 3))
    % end    

    subplot(2+tiene_distancia,1,1)
    plot(movmean(total_reward_per_episode(min_episode_representation:max_episode_representation),movmean_const))
    hold on
    subplot(2+tiene_distancia,1,2)
    plot(movmean(total_duration_per_episode(min_episode_representation:max_episode_representation),movmean_const))
    hold on
    if tiene_distancia
        subplot(3,1,3)
        plot(movmean(distancias_siguiendo_pared(min_episode_representation:max_episode_representation),movmean_const))
        hold on
    end
end

subplot(2+tiene_distancia,1,1)
legend(versiones)
title('Recompensa obtenida por episodio (media móvil)')
xlabel('Episodio')
ylabel('Recompensa')
subplot(2+tiene_distancia,1,2)
legend(versiones)
title('Tiempo sobrevivido obtenida por episodio (media móvil)')
xlabel('Episodio')
ylabel('Tiempo (s)')
if tiene_distancia
    subplot(3,1,3)
    legend(versiones)
    title('Distancia siguiendo la pared (media móvil)')
    xlabel('Episodio')
    ylabel('Distancia (m)')
end

% if ver_estados_descubiertos
%     subplot(3,1,3)
%     legend(versiones)
%     title('Parejas estado-accion descubiertas cada episodio')
% end


hold off

function [total_pairs_discovered_per_episode] = count_discoverings(version, carpeta_LearningData)
    load(carpeta_LearningData+'/vO'+version+'/Qlearning_data_vO'+version+'_mas_reciente.mat', 'num_episodes');
    total_pairs_discovered_per_episode = [];

    for j=1:1:num_episodes
        load(carpeta_LearningData+'/vO'+version+'/Qlearning_data_vO'+version+'_'+j+'.mat','Visitas');

        [num_states, num_actions] = size(Visitas);
        total_stateaction_pairs = num_states * num_actions;
        num_parejas_tot_descub = 0;
        for i=1:1:total_stateaction_pairs % cuento las parejas ya visitadas
            if Visitas(i) ~= 0
                num_parejas_tot_descub = num_parejas_tot_descub + 1;
            end
        end

        total_pairs_discovered_per_episode = [total_pairs_discovered_per_episode; num_parejas_tot_descub];
    end
end


