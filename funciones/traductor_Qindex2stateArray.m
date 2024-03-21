function [stateArray] = traductor_Qindex2stateArray(Qindex, stateArrayOpt)
%Para un valor dentro del indice de la tabla Q me devuelve un struct del
%tipo stateArray donde dice a que estado es equivalente

    Qindex = Qindex-1; % Los arrays en matlab empiezan por 1

    % Array donde pone la base o tamaño de cada discretizacion de variable
    % continua. Algo asi como [4 3 3 4 3 3]
    digit_base_array = [stateArrayOpt.ang_der_length, stateArrayOpt.dist_der_length, stateArrayOpt.qual_der_length, ...
                        stateArrayOpt.ang_fron_length, stateArrayOpt.dist_fron_length, stateArrayOpt.qual_fron_length];

    % Array donde pongo los numeros de cada estado
    for i=length(digit_base_array):-1:1
        digit_state_array(i) = mod(Qindex,digit_base_array(i));
        Qindex = fix(Qindex/digit_base_array(i));
    end

    % Introduzco el digit_state_array en el struct stateArray
    stateArray.ang_der = digit_state_array(1);
    stateArray.dist_der = digit_state_array(2);
    stateArray.qual_der = digit_state_array(3);                 
    stateArray.ang_fron = digit_state_array(4);
    stateArray.dist_fron = digit_state_array(5);
    stateArray.qual_fron = digit_state_array(6);

end

