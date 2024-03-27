clear; close all;

%----------------------------------
% Parámetros a elegir
%----------------------------------
% Booleanos
modo_politica_aprendida = false; % pongo epsilon igual a 5%
ver_simulador = false;
ver_vista_desde_robot = false; % elegir entre ver simulador o ver desde robot
estoy_aprendiendo = true;

ver_recompensas_dentro_episodio = false;
ver_seguimiento_pared_dentro_episodio = false;

ver_evolucion_recompensas = true;

% Enteros 
episodios_totales_entrenamiento = 2000;
num_episodios_tendencia = 100;

% Variables directorio de guardado
carpeta_LearningData = "LearningData12_searchingGoodPerformance";

% CAMBIAR EN CADA PARALELIZACION --------------
stateArrayOpt.num_ojos = 6;
v_apren = "4"; % version del aprendizaje
es_primer_episodio = false;
uso_laser_central = false;
laseres_en_region_izquierda = false;
usar_tiempo_accion_modificada = true;
% ---------------------------------------------

alpha_type = "descenso_infinito"; % descenso_infinito(I) - tiempo_vida(T) - constante(C)
epsilon_type = "ptje_aprendido"; % ptje_aprendido(P) - tiempo_vida(T) - constante(C)
num_pto_topografico = 30; % solo usado en versiones vw y T

%----------------------------------
% Inicialización del entorno de aprendizaje
%----------------------------------

[paredes_mapa, stateArrayOpt, QLearningOpt, Options] = inicializacionConstantes(stateArrayOpt, laseres_en_region_izquierda);

% Parametros del aprendizaje
gamma = QLearningOpt.discount_factor;

[Qtable, Visitas, total_reward_per_episode, total_duration_per_episode, distancias_siguiendo_pared, num_episodes, ptos_aleat] = obtengoQTableYResultadosEpisodios(es_primer_episodio, stateArrayOpt, carpeta_LearningData, v_apren);
[vlin, vang, TL, TG] = obtengoVelocidadesYTiemposDeAccion(carpeta_LearningData, num_pto_topografico, usar_tiempo_accion_modificada);

% Anado las rutinas de calculo al path
addpath("graficos\")
addpath("funciones\")

%----------------------------------
% Inicialización del simulador
%----------------------------------

s = Robot(ver_simulador);

s.defineWalls(paredes_mapa);
s.changeDrawEnv([-0.25 -0.25 10.25 10.25],1); % Defino el tamaño del 'mapa de simulación'
s.setRangeDrawing(0);

%----------------------------------
% Entreno un numero determinado de episodios
%----------------------------------
while num_episodes < episodios_totales_entrenamiento

    [ptje_parejas_no_visitadas] = mensajesIniciales(num_episodes, v_apren, num_pto_topografico, carpeta_LearningData, Visitas, Options, stateArrayOpt, usar_tiempo_accion_modificada, TL, TG);
    epsilon = get_epsilon(epsilon_type, num_episodes, ptje_parejas_no_visitadas, modo_politica_aprendida);

    [robotdef] = inicializarRobotYColocarloEnMapa(s, ptos_aleat);

    % Inicializar variables
    sars = [];
    gtposes = [];
    lasers = [];
    ts = [];
    steptime = 0.032;
   
    t_end_last_episode = s.getSimTime();

    total_reward_episode = 0;
    distancia_siguiendo_pared = 0;

    % Obtengo datos y dibujo
    s.simulate(steptime); 
    [c, iw] = detectoYDibujoColision(s, ver_simulador);    
    [h, t, ts, zs, as, lasers, gtposes] = simuloUnInstanteDeTiempo(ts, gtposes, lasers, s);
    lastst = t;
    
    % Bucle de cada episodio
    while(~c && (t-t_end_last_episode)<QLearningOpt.max_time_duration_episode) %mientras no me choque y no se pase el tiempo maximo
        % ALGORITMO Q-LEARNING
        % Obtengo datos de los sensores, represento y lo paso a Qindex
        [~, stateArrayDisc] = get_stateArrays(zs, as, stateArrayOpt, Options, uso_laser_central);
        current_state = traductor_stateArray2Qindex(stateArrayDisc,stateArrayOpt);

        % Decidir accion
        [V,Vindex] = max(Qtable,[],2);
        politica_actual = getPoliticaActual(Vindex,stateArrayOpt);
        action = next_action_selection(epsilon, politica_actual, current_state, stateArrayOpt);

        % Ejecuto una accion durante un tiempo
        [c, zs, as, lasers, gtposes, t, lastst] = ejecutoAccionDuranteUnTiempo(action, lastst, vlin, vang, TL, TG, t, c, s, steptime, ver_simulador, ts, gtposes, lasers, stateArrayOpt);    

        [stateArrayCont, stateArrayDisc] = get_stateArrays(zs, as, stateArrayOpt, Options, uso_laser_central);
        representarVistaDesdeRobot(ver_vista_desde_robot, stateArrayDisc, stateArrayCont, stateArrayOpt, gtposes(:,end), paredes_mapa);
        next_state = traductor_stateArray2Qindex(stateArrayDisc,stateArrayOpt);

        [distancia_siguiendo_pared, reward, total_reward_episode] = obtengoRecompensas(zs, as, Options, action, c, sars, TL, distancia_siguiendo_pared, total_reward_episode, vlin);
               
        [Qtable, Visitas] = aprendoYActualizoQtable(Qtable, Visitas, current_state, action, reward, next_state, alpha_type, gamma, num_episodes);

        sars = [sars; current_state, action, reward, next_state];
        current_state = next_state;
        
        pruebaRecompensasYSeguimientoParedDentroEpisodio(ver_recompensas_dentro_episodio, ver_seguimiento_pared_dentro_episodio, t, total_reward_episode);

    end
    s.clearFigure()
    total_reward_per_episode = [total_reward_per_episode; total_reward_episode];
    total_duration_per_episode = [total_duration_per_episode; t-t_end_last_episode];
    distancias_siguiendo_pared = [distancias_siguiendo_pared; distancia_siguiendo_pared];
    disp('Reward episodio: '+string(total_reward_episode))

    mostrarEvolucionRecompensas(ver_evolucion_recompensas, total_reward_per_episode, total_duration_per_episode, distancias_siguiendo_pared, num_episodios_tendencia);

    %----------------------------------
    % Recopilación de datos para tener un historial
    %----------------------------------
    if(estoy_aprendiendo)
        num_episodes = length(total_reward_per_episode);
        save('LearningData_Folders/'+carpeta_LearningData+'/vO'+v_apren+'/Qlearning_data_vO'+v_apren+'_'+string(num_episodes)+'.mat', 'Qtable', 'Visitas', ...
            'total_reward_per_episode', 'total_duration_per_episode', 'distancias_siguiendo_pared', ...
            'sars', 'politica_actual', 'gtposes');
        % Guardo todos los datos mas recientes para poder seguir entrenando de
        % forma continua
        save('LearningData_Folders/'+carpeta_LearningData+'/vO'+v_apren+'/Qlearning_data_vO'+v_apren+'_mas_reciente.mat', 'Qtable', 'Visitas', ...
            'total_reward_per_episode', 'total_duration_per_episode', 'distancias_siguiendo_pared', ...
            'num_episodes', 'sars', 'politica_actual');
    end
end


% ---------------------------------------------------------------------------------------------------------------------------------------
% Funciones auxiliares
% ---------------------------------------------------------------------------------------------------------------------------------------


% Inicializo las constantes
function [paredes_mapa, stateArrayOpt, QLearningOpt, Options] = inicializacionConstantes(stateArrayOpt, laseres_en_region_izquierda)
    % Opciones del Q-Learning
    QLearningOpt.max_time_duration_episode = 40; %seconds
    QLearningOpt.discount_factor = 0.99;
    QLearningOpt.learning_rate = 0.1;
    QLearningOpt.exploration_factor = 0.1;

    % Opciones de simulacion
    Options.max_laser_range = 3.3; %maxima distancia tomada como valida (en m)
    Options.number_of_laser_regions = 3; % para calcular el ransac de la pared derecha
    if(laseres_en_region_izquierda)
        Options.total_points_laser = 1080; % si quiero que coja los 3 sectores de visión
    else
        Options.total_points_laser = 2*1080/3; % si quiero que solo coja el sector derecho y frontal
    end

    Options.tolerance_dist_RANSAC = 0.01;
    Options.sampleSize_RANSAC = 2;

    % Defino las caracteristicas de mi struct de estados discretizados
    stateArrayOpt.num_distancias = 3;

    stateArrayOpt.total_states = stateArrayOpt.num_distancias^stateArrayOpt.num_ojos; %: numero de estados posibles
    stateArrayOpt.num_actions = 3;

    stateArrayOpt.min_dist = 0.35;
    stateArrayOpt.max_dist = 1.5;

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
end

% Muestro unos mensajes al inicio de cada episodio
function [ptje_parejas_no_visitadas] = mensajesIniciales(num_episodes, v_apren, num_pto_topografico, carpeta_LearningData, Visitas, Options, stateArrayOpt, usar_tiempo_accion_modificada, TL, TG)
    disp('Episodio '+string(num_episodes)+' del entrenamiento actual')
    disp('Aprendizaje para ' + string(v_apren))
    disp('Numero de puntos laser: '+ string(Options.total_points_laser))
    if carpeta_LearningData == "LearningData_vw"
        disp('Velocidad lineal: '+ string(v_samples(num_pto_topografico)) + ' m/s')
        disp('Velocidad angular: '+ string(w_samples(num_pto_topografico)) + ' º/s')
    elseif usar_tiempo_accion_modificada == true
        disp('Tiempo de mov. lineal TL: '+ string(TL) + ' m/s')
        disp('Tiempo de mov. giro TG: '+ string(TG) + ' º/s')
    end

    % Muestro algunas estadisticas
    reparto_visitas_por_accion = sum(Visitas)
    total_stateaction_pairs = stateArrayOpt.total_states * stateArrayOpt.num_actions;
    num_estados_no_visitados = 0;
    for i=1:1:total_stateaction_pairs
        if sum(Visitas(i)) == 0
            num_estados_no_visitados = num_estados_no_visitados + 1;
        end
    end
    ptje_parejas_no_visitadas = num_estados_no_visitados/total_stateaction_pairs;
    disp('Estados no visitados: '+ string(ptje_parejas_no_visitadas*100)+ '%')
end

function [Qtable, Visitas, total_reward_per_episode, total_duration_per_episode, distancias_siguiendo_pared, num_episodes, ptos_aleat] = obtengoQTableYResultadosEpisodios(es_primer_episodio, stateArrayOpt, carpeta_LearningData, v_apren)
    load('assets/ptos_aleat.mat', 'ptos_aleat');
    
    if es_primer_episodio
        % % Para la primera inicialización cuando cambio de vesion
        Qtable = zeros(stateArrayOpt.total_states,stateArrayOpt.num_actions);
        Visitas = zeros(stateArrayOpt.total_states,stateArrayOpt.num_actions);
        total_reward_per_episode = [];
        total_duration_per_episode = [];
        distancias_siguiendo_pared = [];
        num_episodes = 0;
        
    else  % Importo la versión de las matrices actuales más entrenadas
        load('LearningData_Folders/'+carpeta_LearningData+'/vO'+v_apren+'/Qlearning_data_vO'+v_apren+'_mas_reciente.mat', 'Qtable', 'Visitas', 'total_reward_per_episode', 'total_duration_per_episode', 'distancias_siguiendo_pared', 'num_episodes');
    end
end

function [vlin, vang, TL, TG] = obtengoVelocidadesYTiemposDeAccion(carpeta_LearningData, num_pto_topografico, usar_tiempo_accion_modificada)
    % Inicializo los valores v, w, TL y TG a sus valores predefinidos (cambio en función del entrenamiento)
    vlin = 1;
    vang = pi/2;
    TL = 0.3;
    TG = 0.3;

    if carpeta_LearningData == "LearningData_vw"
        load('LearningData_Folders/LearningData_vw/datos_topograficos.mat', 'v_samples', 'w_samples');

        vlin = v_samples(num_pto_topografico);
        vang = w_samples(num_pto_topografico);
    elseif usar_tiempo_accion_modificada == true
        load('assets/datos_topograficos.mat', 'TL_samples', 'TG_samples');

        vlin = 85/100*0.7; % velocidad lineal, entre 0 y 0.7m/s
        vang = deg2rad( 85/100*180 ); % velocidad angular, entre 36 y 180º/s
        TL = TL_samples(num_pto_topografico);
        TG = TG_samples(num_pto_topografico); 
    end
end

% Inicializo el robot y lo coloco en el mapa
function [robotdef] = inicializarRobotYColocarloEnMapa(s, ptos_aleat)
    % Inicializar robot
    robotdef = s.getRobotDef();

    random_initial_orient = deg2rad( randi([0,360],1) );
    random_initial_pos = randi([1,length(ptos_aleat)],1);
    random_initial_x = ptos_aleat(random_initial_pos,1);
    random_initial_y = ptos_aleat(random_initial_pos,2);

    s.changePose([random_initial_x,random_initial_y,random_initial_orient].');
end

% Detecto si hay colisión, la pinto y aviso
function [c,iw] = detectoYDibujoColision(s, ver_simulador)
    [c,iw] = s.collision(); % c indica si hay colision estando a 1, iw es el indice de la pared que colisiona
        if c && ver_simulador % si hay colision
            figure(s.getFigure());
            ws = s.getWalls();
            hold on;
            plot(ws(iw,[1 3]),ws(iw,[2 4]),'r-','LineWidth',2);
        end 
end

function [h, t, ts, zs, as, lasers, gtposes] = simuloUnInstanteDeTiempo(ts, lasers, gtposes, s)
    t = s.getSimTime();
    ts = [ts s.getSimTime()];

    gtpose = s.getPose();
    gtposes = [gtposes gtpose];

    [zs,as] = s.readRange(); % zs son las distancias de los láseres y as es el angulo con respecto al robot
    lasers = [lasers ; zs];

    h = s.getFigure();
    if ~isempty(h)
        figure(h);
        hold on;
        if (~isempty(gtposes))
            plot(gtposes(1,:),gtposes(2,:),'b-');
        end
    end
    
end

function [c, zs, as, lasers, gtposes, t, lastst] = ejecutoAccionDuranteUnTiempo(action, lastst, vlin, vang, TL, TG, t, c, s, steptime, ver_simulador, ts, gtposes, lasers, stateArrayOpt)
    finish_action = 0;
    while(finish_action == 0 && ~c)
        % Introduzco una accion de 1 a 3 y la ejecuto
    
        if(action==1 && t-lastst<=TL) % Accion 1: Avanza 0.3m
            s.setSpeeds(vlin,0);
        elseif(action==2 && t-lastst<=TG) % Accion 2: Gira a la izquierda unos 23º
            s.setSpeeds(0,vang);
        elseif(action==3 && t-lastst<=TG) % Accion 3: Gira a la derecha unos 23º
            s.setSpeeds(0,-vang);
        elseif(action==0 || action==stateArrayOpt.num_actions+1)
            error('Accion seleccionada no es válida.')
        else
            finish_action = 1;
        end 

        % Obtengo datos y dibujo
        s.simulate(steptime);            
        [c, ~] = detectoYDibujoColision(s, ver_simulador);
        [~, t, ts, zs, as, lasers, gtposes] = simuloUnInstanteDeTiempo(ts, lasers, gtposes, s);
    end
    lastst = t;
end

function [Qtable, Visitas] = aprendoYActualizoQtable(Qtable, Visitas, current_state, action, reward, next_state, alpha_type, gamma, num_episodes)
    % Cambio alpha en función de las veces visitada la pareja estado-accion
    num_visita_pareja = Visitas(current_state,action) + 1; % sumo 1 para que cuando aun no se ha visitado sea 1 en vez de 0
    alpha = get_alpha(alpha_type, num_episodes, num_visita_pareja);

    % Actualizar Q-table
    V_next = max(Qtable(next_state,:));
    Qtable(current_state,action) = Qtable(current_state,action) + alpha * ( reward + gamma*V_next - Qtable(current_state,action) );
    Visitas(current_state,action) = Visitas(current_state,action) + 1;
end

function [distancia_siguiendo_pared, reward, total_reward_episode] = obtengoRecompensas(zs, as, Options, action, c, sars, TL, distancia_siguiendo_pared, total_reward_episode, vlin)
    % Obtengo datos del ransac de la pared de la derecha
    [~, ~, ~, ~, triplete_der] = RANSAC_triplete_der(zs, as, Options);
            
    % Obtengo recompensa
    [reward, estoy_siguiendo_pared] = reward_discretization(action, c, sars, triplete_der); 
    if estoy_siguiendo_pared
        distancia_siguiendo_pared = distancia_siguiendo_pared + vlin*TL;
    end 
    total_reward_episode = total_reward_episode + reward;
end

function mostrarEvolucionRecompensas(ver_evolucion_recompensas, total_reward_per_episode, total_duration_per_episode, distancias_siguiendo_pared, num_episodios_tendencia)
    if(ver_evolucion_recompensas)
        figure(4)
        clf
        subplot(3,1,1)
        hold on
        title('Recompensa total por episodio')
        plot(total_reward_per_episode,'-b')
        plot(movmean(total_reward_per_episode,num_episodios_tendencia), '.-k')
        hold off
        subplot(3,1,2)
        title('Tiempo sobrevivido por episodio')
        hold on
        plot(total_duration_per_episode,'-r')
        plot(movmean(total_duration_per_episode,num_episodios_tendencia), '.-k')
        subplot(3,1,3)
        title('Distancia siguiendo pared')
        hold on
        plot(distancias_siguiendo_pared,'-g')
        plot(movmean(distancias_siguiendo_pared,num_episodios_tendencia), '.-k')
        hold off
    end
end

function pruebaRecompensasYSeguimientoParedDentroEpisodio(ver_recompensas_dentro_episodio, ver_seguimiento_pared_dentro_episodio, t, total_reward_episode)
    if(ver_recompensas_dentro_episodio)
        figure(2)
        hold on
        plot(t,total_reward_episode,'or')
        hold off
    end
    if(ver_seguimiento_pared_dentro_episodio)
        figure(2)
        hold on
        plot(t,distancia_siguiendo_pared,'or')
        hold off
    end
end