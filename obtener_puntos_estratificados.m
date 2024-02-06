clear all; close all;

% Número de puntos deseados por estrato
modo_aleatorio = true;
num_points_per_stratum = 1;
num_sectores_TL = 3;
num_sectores_TG = 2;
min_TL = 0.255;
max_TL = 0.5;
min_TG = 0.01;
max_TG = 0.255;

% División en estratos para v y w
TL_strata = linspace(min_TL, max_TL, num_sectores_TL+1);
TG_strata = linspace(min_TG, max_TG, num_sectores_TG+1);

% Generación de puntos aleatorios estratificados
carpeta_LearningData = "LearningData_T";
load(carpeta_LearningData+'/datos_topograficos.mat', 'TL_samples', 'TG_samples');

for i = 1:length(TL_strata)-1
    for j = 1:length(TG_strata)-1
        if modo_aleatorio == true
            % Generar puntos aleatorios dentro de cada estrato
            TL_rand = TL_strata(i) + (TL_strata(i + 1) - TL_strata(i)) * rand(1, num_points_per_stratum);
            TG_rand = TG_strata(j) + (TG_strata(j + 1) - TG_strata(j)) * rand(1, num_points_per_stratum);
        else
            TL_rand = TL_strata(i) + (TL_strata(i + 1) - TL_strata(i)) * 0.5;
            TG_rand = TG_strata(j) + (TG_strata(j + 1) - TG_strata(j)) * 0.5;
        end
        
        % Agregar los puntos generados a la lista de muestras
        TL_samples = [TL_samples, TL_rand];
        TG_samples = [TG_samples, TG_rand];
    end
end

% Mostrar los puntos aleatorios estratificados generados
scatter(TL_samples, TG_samples);
% xlim([min_TL max_TL]); ylim([min_TG max_TG]);
xlim([0.01 0.5]); ylim([0.01 0.5]);
xlabel('TL');
ylabel('TG');
title('Puntos aleatorios estratificados en TL y TG');

% CUIDADO PORQUE LIMPIA LO YA EXISTENTE
% save(carpeta_LearningData+'/datos_topograficos.mat', 'TL_samples', 'TG_samples');
