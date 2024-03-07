function [stateArrayCont, stateArrayDisc] = get_stateArrays(zs, as, stateArrayOpt, Options, uso_laser_central)
% Devuelve 2 matrices con 2 columnas de valores cada una. 
% En la columna de la izq. devuelve la distancia vista desde cada uno de los ojos.
% En la de la derecha devuelve el angulo en polar que representa la orientación de
% dicho ojo. Esta columna de la derecha siempre posee un valor continuo.
    num_ojos = stateArrayOpt.num_ojos;
    stateArrayCont = zeros([num_ojos, 2]); stateArrayDisc = stateArrayCont;

    size_laser_region = Options.total_points_laser/num_ojos;

    for i=1:1:num_ojos
        % Asegura que los índices sean enteros y estén dentro del rango válido
        min_laser_region = max(1, round((i-1)*size_laser_region + 1));
        max_laser_region = min(length(zs), round(i*size_laser_region));

        if(uso_laser_central)
            % Encuentra el punto situado enmedio de la región
            indice_laser_ojos = (max_laser_region + min_laser_region)/2;
            indice_laser_ojos = round(indice_laser_ojos,0);

            if( zs(indice_laser_ojos) > stateArrayOpt.max_dist )
                stateArrayCont(i,:) = [stateArrayOpt.max_dist, as(indice_laser_ojos)];
            else
                stateArrayCont(i,:) = [zs(indice_laser_ojos), as(indice_laser_ojos)]; 
            end
        else 
            % Encuentra el punto con la distancia mínima en la región
            [min_dist, min_index] = min(zs(min_laser_region:max_laser_region));
            min_angle = as(min_laser_region + min_index - 1); % Ajuste al índice correcto

            if( min_dist > stateArrayOpt.max_dist )
                stateArrayCont(i,:) = [stateArrayOpt.max_dist, min_angle];
            else
                stateArrayCont(i,:) = [min_dist, min_angle];
            end
        end

        stateArrayDisc(i,1) = [obtener_valor_discreto(stateArrayOpt.min_dist, stateArrayOpt.max_dist, stateArrayOpt.num_distancias, stateArrayCont(i,1))];
    end
    
end

function [valor_discreto] = obtener_valor_discreto(min,max,num_discretiz,valor_cont)
% Función que devuelve un valor discreto en la base dada para un valor_cont
% comprendido entre min y max
% min: valor minimo continuo posible de la variable
% max: valor maximo continuo posible de la variable
% num_discretiz: numero de valores discretos que representan la variable
%   continua, es decir, numero de niveles que tiene la variable discretiz.
% valor_discreto: valor discreto de la region que se quiere representar,
%   empieza por 0 y acaba en num_discretiz-1 

    rango_region = (max-min)/num_discretiz;
    
    valor_discreto = fix((valor_cont-min)/rango_region);
    if(valor_discreto==num_discretiz) % para que no se salga de rango en el valor máximo
        valor_discreto = num_discretiz-1;
    end
end
