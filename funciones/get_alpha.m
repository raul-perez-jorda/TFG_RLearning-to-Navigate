function [alpha] = get_alpha(alpha_type, num_episodes, num_visita_pareja)
    switch alpha_type

        case "descenso_infinito"
            alpha = 1/( num_visita_pareja^0.55 ); 

        case "tiempo_vida"
            duracion_entrenamiento = 300;
            alpha_inicial = 0.5;
            alpha_final = 0.01;
            if(num_episodes < duracion_entrenamiento)
                m = -(alpha_inicial-alpha_final)/(duracion_entrenamiento);
                n = alpha_inicial - m;
                alpha = m*num_episodes + n; % en episodio 1 es 0.5 en episodio 
            else
                alpha = 0.01;
            end

        case "constante"
            alpha = 0.1;

        otherwise
            error('Nombre alpha no valido')
    end            

end