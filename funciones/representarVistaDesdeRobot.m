function representarVistaDesdeRobot(ver_vista_desde_robot, stateArrayDisc, stateArrayCont, stateArrayOpt, gtpose, paredes_mapa)
% Representa en un plot un esquema de como sería la información que ve el robot
    if (ver_vista_desde_robot)
        num_figure = 3;
        hull_lat_dim = 0.354; % tamaño de los lados del robot

        gt_x = gtpose(1);
        gt_y = gtpose(2);
        gt_ang = gtpose(3);

        figure(num_figure)
        clf("reset")
        hold on

        for ojo = 1:1:stateArrayOpt.num_ojos
            ang = stateArrayCont(ojo,2);
            ang = gt_ang + ang;

            % Represento el estado continuo
            % dist = stateArrayCont(ojo,1);
            % plot(dist*cos(ang) + gt_x, dist*sin(ang) + gt_y, '*k')

            % Dibujo todas las posibles discretizaciones
            x_full_range = [stateArrayOpt.min_dist*cos(ang),stateArrayOpt.max_dist*cos(ang)] + gt_x;
            y_full_range = [stateArrayOpt.min_dist*sin(ang),stateArrayOpt.max_dist*sin(ang)] + gt_y;
            plot( x_full_range, y_full_range, 'ob', x_full_range, y_full_range, '-b')

            % Represento el estado discreto
            [dist_min, dist_max] = obtener_valor_cont_maxmin_region(stateArrayOpt.min_dist, stateArrayOpt.max_dist, stateArrayOpt.num_distancias, stateArrayDisc(ojo,1));

            x = [dist_min*cos(ang),dist_max*cos(ang)] + gt_x;
            y = [dist_min*sin(ang),dist_max*sin(ang)] + gt_y;

            plot(x, y, 'ob', 'LineWidth', 1.5)
            plot(x, y, '-b', 'LineWidth', 1.5)
        end

        % Pinto el robot
        initial_x_robot = [-hull_lat_dim/2 -hull_lat_dim/2 hull_lat_dim/2 hull_lat_dim/2];
        initial_y_robot = [hull_lat_dim/2 -hull_lat_dim/2 -hull_lat_dim/2 hull_lat_dim/2];
        x_robot_body = (initial_x_robot*cos(gt_ang) - initial_y_robot*sin(gt_ang) ) + gt_x;
        y_robot_body = (initial_x_robot*sin(gt_ang) + initial_y_robot*cos(gt_ang) ) + gt_y;
        robot = polyshape(x_robot_body,y_robot_body);

        initial_x_robot = [hull_lat_dim/2 hull_lat_dim/2 hull_lat_dim];
        initial_y_robot = [-hull_lat_dim/4 hull_lat_dim/4 0];
        x_robot_head = (initial_x_robot*cos(gt_ang) - initial_y_robot*sin(gt_ang) ) + gt_x;
        y_robot_head = (initial_x_robot*sin(gt_ang) + initial_y_robot*cos(gt_ang) ) + gt_y;
        robot_head = polyshape(x_robot_head,y_robot_head);

        plot(robot)
        plot(robot_head)

        % Pinto las paredes
        pintoParedesMapa(paredes_mapa);

        grid on
        xlim([-1.25 11.25]); ylim([-1.25 11.25]);
        hold off;
    end
end

function [valor_min_region, valor_max_region] = obtener_valor_cont_maxmin_region(min,max,num_discretiz,valor_discreto)
% Función que devuelve un valor enmedio de la región continua discretizada representativo del valor discreto dado.
% min: valor minimo continuo posible de la variable
% max: valor maximo continuo posible de la variable
% num_discretiz: numero de valores discretos que representan la variable
%   continua, es decir, numero de niveles que tiene la variable discretiz.
% valor_discreto: valor discreto de la region que se quiere representar,
%   empieza por 0 y acaba en num_discretiz-1 
    rango_region = (max-min)/(num_discretiz);
    valor_min_region = min + valor_discreto * rango_region;
    valor_max_region = min + (valor_discreto+1) * rango_region;
end
