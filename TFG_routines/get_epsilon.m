function [epsilon] = get_epsilon(epsilon_type, num_episodes, ptje_parejas_no_visitadas)
    switch epsilon_type

        case "ptje_aprendido"
            epsilon = 0.05 + ptje_parejas_no_visitadas*0.25; 

        case "tiempo_vida"
            duracion_entrenamiento = 300;
            epsilon_inicial = 1;
            epsilon_final = 0.01;
            if(num_episodes < duracion_entrenamiento)
                m = -(epsilon_inicial-epsilon_final)/(duracion_entrenamiento);
                n = epsilon_inicial - m;
                epsilon = m*num_episodes + n; % en episodio 1 es 0.5 en episodio 
            else
                epsilon = 0.01;
            end

        case "constante"
            epsilon = 0.1;

        otherwise
            error('Nombre epsilon no valido')
    end            

end