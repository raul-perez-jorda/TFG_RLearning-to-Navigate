clear; close all;

carpeta_LearningData = "LearningData_Folders/LearningData12_searchingGoodPerformance";
version = "2";

siempre_mismo_inicio = true;
x_inicial = 5;
y_inicial = 5;

anchoMin = 0.5; % Ancho mínimo del trazo
anchoMax = 2; % Ancho máximo del trazo

figure(1)
hold on
axis equal

paredes_mapa = [0, 0, 10, 0 ;... % Defino el inicio y el final de cada pared
                    6, 0, 6, 3 ; ...
                    10, 0, 10, 10 ; ...
                    10, 10, 0, 10 ; ...
                    0, 10, 0, 0 ; ...
                    0, 2, 3, 2 ; ...
                    6, 4, 10, 4;
                    7, 4, 7, 6; ...
                    6 6 7 6; ...
                    6 6 6 7; ...
                    7 8 10 8; ...
                    6 8 6 10; ...
                    0 7 1 7; ...
                    4 7 3 7; ...
                    3 7 3 4; ...
                    3 4 4 4; ...
                        4 4 4 7];
pintoParedesMapa(paredes_mapa);


load(carpeta_LearningData+'/vO'+version+'/Qlearning_data_vO'+version+'_mas_reciente.mat','num_episodes');
for i = 1:num_episodes
    load(carpeta_LearningData+'/vO'+version+'/Qlearning_data_vO'+version+'_'+i+'.mat','gtposes');

    error_x_inicial = abs(x_inicial - gtposes(1,1));
    error_y_inicial = abs(y_inicial - gtposes(2,1));

    if(error_x_inicial < 0.2 && error_y_inicial < 0.2 || siempre_mismo_inicio==false)
        % Interpola el color desde rojo (1, 0, 0) a verde (0, 1, 0)
        color = [1 - (i-1)/(num_episodes-1), (i-1)/(num_episodes-1), 0];
        
        % Calcula el ancho del trazo basado en el episodio actual
        anchoTrazo = anchoMin + (anchoMax - anchoMin) * (i-1) / (num_episodes-1);
        
        plot(gtposes(1,:), gtposes(2,:), 'Color', color, 'LineWidth', anchoTrazo);        
    end
end

hold off
