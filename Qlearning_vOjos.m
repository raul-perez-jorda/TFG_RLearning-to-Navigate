clear; close all;

%----------------------------------
% Parámetros a elegir
%----------------------------------
% Booleanos
modo_politica_aprendida = false; % pongo epsilon igual a 5%
ver_simulador = true;
ver_vista_desde_robot = false; % elegir entre ver simulador o ver desde robot
estoy_aprendiendo = false;

ver_recompensas_dentro_episodio = false;
ver_seguimiento_pared_dentro_episodio = false;
ver_evolucion_recompensas = false;

% Enteros
episodios_totales_entrenamiento = 1000;
num_episodios_tendencia = 30;

% Variables directorio de guardado
carpeta_LearningData = "LearningData_T";

% CAMBIAR EN CADA PARALELIZACION --------------
v_apren = "6D_IP_T18"; % version del aprendizaje
num_pto_topografico = 18; % solo usado en versiones vw y T
es_primer_episodio = true;
% ---------------------------------------------

alpha_type = "descenso_infinito"; % descenso_infinito(I) - tiempo_vida(T) - constante(C)
epsilon_type = "ptje_aprendido"; % ptje_aprendido(P) - tiempo_vida(T) - constante(C)

%----------------------------------
% Inicialización del entorno de aprendizaje
%----------------------------------

[paredes_mapa, stateArrayOpt, Options, QLearningOpt] = inicializacionConstantes();

% Parametros del aprendizaje
alpha = QLearningOpt.learning_rate;
gamma = QLearningOpt.discount_factor;
epsilon = QLearningOpt.exploration_factor;

if es_primer_episodio
    % % Para la primera inicialización cuando cambio de vesion
    Qtable = zeros(stateArrayOpt.total_states,stateArrayOpt.num_actions);
    Visitas = zeros(stateArrayOpt.total_states,stateArrayOpt.num_actions);
    total_reward_per_episode = [];
    total_duration_per_episode = [];
    distancias_siguiendo_pared = [];
    num_episodes = 0;
    load('LearningData_Folders/'+carpeta_LearningData+'/ptos_aleat.mat', 'ptos_aleat');
else  % Importo la versión de las matrices actuales más entrenadas
    load('LearningData_Folders/'+carpeta_LearningData+'/vO'+v_apren+'/Qlearning_data_vO'+v_apren+'_mas_reciente.mat', 'ptos_aleat', 'Qtable', 'Visitas', 'total_reward_per_episode', 'total_duration_per_episode', 'distancias_siguiendo_pared', 'num_episodes');
end

% Anado las rutinas de calculo al path
addpath("funciones\")

% Inicializo los valores v, w, TL y TG a sus valores predefinidos (cambio en función del entrenamiento)
vlin = 1;
vang = pi/2;
TL = 0.3;
TG = 0.3;

if carpeta_LearningData == "LearningData_vw"
    load('LearningData_Folders/LearningData_vw/datos_topograficos.mat', 'v_samples', 'w_samples');

    vlin = v_samples(num_pto_topografico);
    vang = w_samples(num_pto_topografico);
elseif carpeta_LearningData == "LearningData_T" || carpeta_LearningData=="LearningData"
    load('LearningData_Folders/LearningData_T/datos_topograficos.mat', 'TL_samples', 'TG_samples');

    vlin = 85/100*0.7; % velocidad lineal, entre 0 y 0.7m/s
    vang = deg2rad( 85/100*180 ); % velocidad angular, entre 36 y 180º/s
    TL = TL_samples(num_pto_topografico);
    TG = TG_samples(num_pto_topografico); 
end

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

    mensajesIniciales()    

    epsilon = get_epsilon(epsilon_type, num_episodes, ptje_parejas_no_visitadas, modo_politica_aprendida);

    [robotdef] = inicializarRobotYColocarloEnMapa();

    % Inicializar variables
    sars = [];
    gtposes = [];
    lasers = [];
    
    encs = [];
    motors = [];
    ts = [];
    st = 0;
    i_st = 1; % indice de estados para secuencia de movimientos
    oldencs = [s.readEncoder('L') s.readEncoder('R')]; % degs
    oldp = s.getPose();
    finish = 0;
    ind = 1;
    t0 = tic;
    steptime = 0.032;
   
    c=0;
    t = s.getSimTime();
    t_end_last_episode = s.getSimTime();

    total_reward_episode = 0;
    distancia_siguiendo_pared = 0;
    
    % Bucle de cada episodio
    while(~c && (t-t_end_last_episode)<QLearningOpt.max_time_duration_episode) %mientras no me choque y no se pase el tiempo maximo
        % Obtengo datos y dibujo
        s.simulate(steptime);
 
        [c, iw] = detectoYDibujoColision();
        
        ts = [ts s.getSimTime()];
        [zs,as] = s.readRange();
        lasers = [lasers ; zs];
        encs = [encs ; s.readEncoder('L') s.readEncoder('R')];
        motors = [motors ; s.getMotor('L') s.getMotor('R')];
        h = s.getFigure();
        if ~isempty(h)
            figure(h);
            hold on;
            if (~isempty(gtposes))
                plot(gtposes(1,:),gtposes(2,:),'b-');
            end
        end
        t = s.getSimTime();
        lastst = t;
        gtpose = s.getPose();
        gtposes = [gtposes gtpose];
        % next step
        oldencs = [s.readEncoder('L') s.readEncoder('R')];    
        ind = ind + 1;

        % ALGORITMO Q-LEARNING
        % Obtengo datos de los sensores, represento y lo paso a Qindex
        [stateArrayCont, stateArrayDisc] = get_stateArrays(zs, as, stateArrayOpt, Options);
        current_state = traductor_stateArray2Qindex(stateArrayDisc,stateArrayOpt);

        % Decidir accion y ejecutarla
        [V,Vindex] = max(Qtable,[],2);
        politica_actual = getPoliticaActual(Vindex,stateArrayOpt);
        action = next_action_selection(epsilon,politica_actual,current_state,stateArrayOpt);
        %disp('Accion: '+string(action))

        finish_action = 0;
        while(finish_action == 0 && ~c)
            % Introduzco una accion de 1 a 3 y la ejecuto
        
            if(action==1 && t-lastst<=TL) % Accion 1: Avanza 0.3m
                s.setSpeeds(vlin,0);
            elseif(action==2 && t-lastst<=TG) % Accion 2: Gira a la izquierda unos 23º
                s.setSpeeds(0,vang);
            elseif(action==3 && t-lastst<=TG) % Accion 3: Gira a la derecha unos 23º
                s.setSpeeds(0,-vang);
            elseif(action==-1 || action==stateArrayOpt.num_actions+1)
                error('Accion seleccionada no es válida.')
            else
                finish_action = 1;
                %disp(string(current_state)+'->'+string(next_state))
            end 

            % Obtengo datos y dibujo
            s.simulate(steptime);
            [c,iw] = s.collision();
            if c && ver_simulador % si hay colision
                figure(s.getFigure());
                ws = s.getWalls();
                hold on;
                plot(ws(iw,[1 3]),ws(iw,[2 4]),'r-','LineWidth',2);
            end 
            ts = [ts s.getSimTime()];
            [zs,as] = s.readRange();
            lasers = [lasers ; zs];
            encs = [encs ; s.readEncoder('L') s.readEncoder('R')];
            motors = [motors ; s.getMotor('L') s.getMotor('R')];
            h = s.getFigure();
            if ~isempty(h)
                figure(h);
                hold on;
                if (~isempty(gtposes))
                    plot(gtposes(1,:),gtposes(2,:),'b-');
                end
            end
            t = s.getSimTime();
            gtpose = s.getPose();
            gtposes = [gtposes gtpose];
            % next step
            oldencs = [s.readEncoder('L') s.readEncoder('R')];    
            ind = ind + 1;

            % Obtengo datos del ransac de la pared de la derecha
            maxDistance = 0.01;
            sampleSize = 2;
            [ransac_x, ransac_y, m, n, triplete_der] = RANSAC_triplete_der(zs, as, sampleSize, maxDistance, Options);

            % Obtengo datos del siguiente estado
            [next_stateArrayCont, next_stateArrayDisc] = get_stateArrays(zs, as, stateArrayOpt, Options);
            next_state = traductor_stateArray2Qindex(next_stateArrayDisc,stateArrayOpt);
    
            % Compruebo si hay colision
            s.simulate(steptime);
            [c,iw] = s.collision(); % compruebo si hay colision

            if(ver_vista_desde_robot)
                representa_stateArray(stateArrayDisc, stateArrayCont,stateArrayOpt, gtpose, ransac_x, ransac_y)
            end
        end
        finish_action = 0;
        
        % Obtengo recompensa
        [reward, estoy_siguiendo_pared] = reward_discretization(action,c, sars, triplete_der, TL, TG); 
        if estoy_siguiendo_pared
            distancia_siguiendo_pared = distancia_siguiendo_pared + vlin*TL;
        end 
        total_reward_episode = total_reward_episode + reward;
               

        % Cambio alpha en función de las veces visitada la pareja
        % estado-accion
        num_visita_pareja = Visitas(current_state,action)+1; % sumo 1 para que cuando aun no se ha visitado sea 1 en vez de 0
        alpha = get_alpha(alpha_type, num_episodes, num_visita_pareja);
    
        % Actualizar Q-table
        V_next = max(Qtable(next_state,:));
        Qtable(current_state,action) = Qtable(current_state,action) + alpha * ( reward + gamma*V_next - Qtable(current_state,action) );
        Visitas(current_state,action) = Visitas(current_state,action) + 1;
        sars = [sars; current_state, action, reward, next_state];

        current_state = next_state;

        
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
    s.clearFigure()
    total_reward_per_episode = [total_reward_per_episode; total_reward_episode];
    total_duration_per_episode = [total_duration_per_episode; t-t_end_last_episode];
    distancias_siguiendo_pared = [distancias_siguiendo_pared; distancia_siguiendo_pared];
    disp('Reward episodio: '+string(total_reward_episode))

    mostrarEvolucionRecompensas(ver_evolucion_recompensas)

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
        save('LearningData_Folders/'+carpeta_LearningData+'/vO'+v_apren+'/Qlearning_data_vO'+v_apren+'_mas_reciente.mat', 'ptos_aleat', 'Qtable', 'Visitas', ...
            'total_reward_per_episode', 'total_duration_per_episode', 'distancias_siguiendo_pared', ...
            'num_episodes', 'sars', 'politica_actual', ...
            'TL_samples', 'TG_samples'); % por si acaso borro los iniciales
    end
end


% ---------------------------------------------------------------------------------------------------------------------------------------
% Funciones auxiliares
% ---------------------------------------------------------------------------------------------------------------------------------------


% Inicializo las constantes
function [paredes_mapa, stateArrayOpt, QLearningOpt, Options] = inicializacionConstantes()
    % Opciones del Q-Learning
    QLearningOpt.max_time_duration_episode = 40; %seconds
    QLearningOpt.discount_factor = 0.99;
    QLearningOpt.learning_rate = 0.1;
    QLearningOpt.exploration_factor = 0.1;

    % Opciones de simulacion
    Options.max_laser_range = 3.3; %maxima distancia tomada como valida (en m)
    Options.number_of_laser_regions = 3; % para calcular el ransac de la pared derecha
    % Options.total_points_laser = 1080; % si quiero que coja los 3 sectores de visión
    Options.total_points_laser = 2*1080/3; % si guiero que solo coja el sector derecho y frontal

    % Defino las caracteristicas de mi struct de estados discretizados
    stateArrayOpt.num_ojos = 6;
    stateArrayOpt.num_distancias = 3;

    stateArrayOpt.total_states = stateArrayOpt.num_distancias^stateArrayOpt.num_ojos; %: numero de estados posibles
    stateArrayOpt.num_actions = 3;

    stateArrayOpt.min_dist = 0.4;
    stateArrayOpt.max_dist = 3;

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
function mensajesIniciales()
    disp('Episodio '+string(num_episodes)+' del entrenamiento actual')
    disp('Aprendizaje para ' + string(v_apren))
    disp('Numero de puntos laser: '+ string(Options.total_points_laser))
    if carpeta_LearningData == "LearningData_vw"
        disp('Velocidad lineal: '+ string(v_samples(num_pto_topografico)) + ' m/s')
        disp('Velocidad angular: '+ string(w_samples(num_pto_topografico)) + ' º/s')
    elseif carpeta_LearningData == "LearningData_T"
        disp('Tiempo de mov. lineal TL: '+ string(TL_samples(num_pto_topografico)) + ' m/s')
        disp('Tiempo de mov. giro TG: '+ string(TG_samples(num_pto_topografico)) + ' º/s')
    end

    % Muestro algunas estadisticas
    reparto_visitas_por_accion = sum(Visitas);
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

% Inicializo el robot y lo coloco en el mapa
function [robotdef] = inicializarRobotYColocarloEnMapa()
    % Inicializar robot
    robotdef = s.getRobotDef();

    random_initial_orient = deg2rad( randi([0,360],1) );
    random_initial_pos = randi([1,length(ptos_aleat)],1);
    random_initial_x = ptos_aleat(random_initial_pos,1);
    random_initial_y = ptos_aleat(random_initial_pos,2);

    s.changePose([random_initial_x,random_initial_y,random_initial_orient].');
end

% Detecto si hay colisión, la pinto y aviso
function [c,iw] = detectoYDibujoColision()
    [c,iw] = s.collision();
        if c && ver_simulador % si hay colision
            figure(s.getFigure());
            ws = s.getWalls();
            hold on;
            plot(ws(iw,[1 3]),ws(iw,[2 4]),'r-','LineWidth',2);
        end 
end

function mostrarEvolucionRecompensas(ver_evolucion_recompensas)
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