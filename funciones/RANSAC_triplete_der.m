function [ransac_x, ransac_y, m, n, triplete] = RANSAC_triplete_der(zs, as, Options)  
% Para una nube de puntos dada obtener los puntos de la linea ransac para
% valor x e y; la pendiente y el termino independientes de esta recta m y
% n; finalmente el angulo respecto a la recta eje x, la distancia más corta
% entre esta recta y el punto 0,0 y la cantidad de puntos que son del
% ransac respecto a la cantidad de puntos máxima posible en porcentaje.
%
% ¡¡¡ En caso de no tener ningun punto asigna una recta paralela de
% distancia máxima para los valores de alpha, d y qual

    ptos_cartes = [zs'.*cos(as'), zs'.*sin(as')]; %puntos del sonar en coordenadas cartesianas (respecto del robot)
    points_region = Options.total_points_laser/Options.number_of_laser_regions; %points laser divided # regions
    ptos_cartes1 = ptos_cartes(1:points_region,:); %puntos de la parte derecha del robot

    % Limpiar puntos del laser con distancia mayor a la máxima
    vec_ptos_validos = logical([]);
    for i=1:1:length(zs)
        if(zs(i) > Options.max_laser_range)
            vec_ptos_validos(i,1) = false;
        else
            vec_ptos_validos(i,1) = true;
        end
    end

    points = ptos_cartes1(vec_ptos_validos(1:points_region), :);


    if(length(points)>2)
        fitLineFcn = @(points) polyfit(points(:,1), points(:,2), 1); % fit function using polyfit
        evalLineFcn = ...   % distance evaluation function
          @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);
    
        [~, inlierIdx] = ransac(points,fitLineFcn,evalLineFcn, ...
          Options.sampleSize_RANSAC,Options.tolerance_dist_RANSAC);
    
        modelInliers = polyfit(points(inlierIdx,1),points(inlierIdx,2),1);
    
        %Dibujar interpolacion RANSAC
        inlierPts = points(inlierIdx,:);
        ransac_x = [min(inlierPts(:,1)) max(inlierPts(:,1))];
        ransac_y = modelInliers(1)*ransac_x + modelInliers(2);        
    
        % Obtener datos del angulo respecto a la pared (alpha)
        alpha = atan(modelInliers(1)); % angulo de la pared respecto a la dirección movimiento robot
    
        % Obtener la distancia al punto más cercano de la pared (d)
        syms x
        m = double( modelInliers(1) ); %pendiente de la pared
        n = double( modelInliers(2) ); %termino independiente de la pared
        
        if(abs(cos(alpha))<0.001)
            d = -n/m;
        else
            d = abs(n*cos(alpha));
        end
    
        % Obtener la calidad de dicha recta (qual)
        qual = length(inlierPts)/points_region * 100;  
    else
        ransac_x = [];
        ransac_y = [];
        m = 0;
        n = [];
        d = Options.max_laser_range;
        alpha = 0;
        qual = 0;
    end      

    triplete.alpha = alpha;
    triplete.d = d;
    triplete.qual = qual;
end

