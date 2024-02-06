function [Qindex] = traductor_stateArray2Qindex(stateArray, stateArrayOpt)
    % Function that returns a value of a Qindex in reference to an
    % stateArray
    
    % Array donde pone la base o tamaño de cada discretizacion de variable
    % continua. Algo asi como [4 3 3 4 3 3]
    digit_base_array = stateArrayOpt.num_distancias * ones([1,stateArrayOpt.num_ojos]);

    % Array donde pongo el valor por el cual multiplica dicho stateArray en
    % esa posicion. Es decir, el valor que tomaría en el Qindex si dicho
    % input fuese 0.
    for i=1:1:length(digit_base_array) % relleno los valores del array
        weight = 1;
        for j = i+1 : 1 : length(digit_base_array)
            weight = weight * digit_base_array(j);
        end
        weight_base_array(i) = weight;
    end

    %Array donde pongo los valores numericos del struct stateArray de
    %entrada. Algo así como [0 2 2 1 1 0]
    digit_state_array = stateArray(:,1)'; 

    Qindex = 0;
    for i=1:1:length(digit_base_array)
        Qindex = Qindex + digit_state_array(i)*weight_base_array(i);
    end

    Qindex = Qindex + 1; % Los arrays en matlab empiezan por el 1
end