function [politica_actual] = getPoliticaActual(Vindex, stateArrayOpt)
% Dado un Vindex o vector de los indices de la mejor accion devuelve un
% array donde cada fila es un estado y el valor es la mejor accion discreta
% dado dicho estado. Este vector se llama politica
    politica_actual = zeros(length(Vindex),1);

    for i=1:1:length(Vindex)
        politica_actual(i) = mod( (Vindex(i)-1), stateArrayOpt.num_actions ) + 1;
    end
end