clear; close all;

%----------------------------------
% Parámetros a elegir
%----------------------------------
% Booleanos
ver_simulador = true;
estoy_aprendiendo = true;

ver_evolucion_recompensas = false;

% Enteros
episodios_totales_entrenamiento = 100;
num_episodios_tendencia = 30;

% Variables directorio de guardado
carpeta_LearningData = "LearningData_GPT";

% CAMBIAR EN CADA PARALELIZACION --------------
v_apren = "GPT_1"; % version del aprendizaje
es_primer_episodio = true;
% ---------------------------------------------

%----------------------------------
% Inicialización del entorno de aprendizaje
%----------------------------------

[paredes_mapa, QLearningOpt] = inicializacionConstantes();
load('LearningData_Folders/'+carpeta_LearningData+'/ptos_aleat.mat', 'ptos_aleat'); % Importo los puntos aleatorios desde donde puede iniciar la simulación


if es_primer_episodio
    % % Para la primera inicialización cuando cambio de vesion
    total_reward_per_episode = [];
    total_duration_per_episode = [];
    distancias_siguiendo_pared = [];
    num_episodes = 0;

    [net, DQN_params, trainingOptions] = crearNuevaRedNeuronal();
else  % Importo la versión de las matrices actuales más entrenadas
    load('LearningData_Folders/'+carpeta_LearningData+'/v'+v_apren+'/Qlearning_data_vO'+v_apren+'_mas_reciente.mat', 'total_reward_per_episode', 'total_duration_per_episode', 'distancias_siguiendo_pared', 'num_episodes', 'net', 'DQN_params', 'trainingOptions');
end

% Anado las rutinas de calculo al path
addpath("funciones\")

% Inicializo los valores v, w, TL y TG a sus valores predefinidos (cambio en función del entrenamiento)
vlin = 0.7;
vang = pi/2;
TA = 0.3;

%----------------------------------
% Inicialización del simulador
%----------------------------------

s = Robot(ver_simulador);

s.defineWalls(paredes_mapa); % Defino las paredes
s.changeDrawEnv([-0.25 -0.25 10.25 10.25],1); % Defino el tamaño del 'mapa de simulación'
s.setRangeDrawing(0); % No dibuja los láseres

%----------------------------------
% Entreno un numero determinado de episodios
%----------------------------------
while num_episodes < episodios_totales_entrenamiento

    mensajesIniciales()    

    [robotdef] = inicializarRobotYColocarloEnMapa();

    % Inicializar variables
    gtposes = [];
    lasers = [];    
    ts = [];

    steptime = 0.032;
   
    c = 0; % Inicializo la variable de colisión
    t = s.getSimTime();
    t_end_last_episode = s.getSimTime();

    total_reward_episode = 0;
    distancia_siguiendo_pared = 0;
    
    % Bucle de cada episodio
    while(~c && ( t-t_end_last_episode ) < QLearningOpt.max_time_duration_episode ) %mientras no me choque y no se pase el tiempo maximo
        % Obtengo datos y dibujo
        s.simulate(steptime);
 
        [c, iw] = detectoYDibujoColision();

        [h, t, ts, zs, as, lasers, gtpose, gtposes] = simuloUnInstanteDeTiempo(ts, gtposes, lasers);
        lastst = t;

        % Ejecuto una accion
        finish_action = 0;
        while(finish_action == 0 && ~c)      
            if(t-lastst<=TA) % Accion 1: Avanza 0.3m
                s.setSpeeds(vlin,vang);
            else
                finish_action = 1;
            end 

            % Obtengo datos y dibujo
            s.simulate(steptime);
            [c, iw] = detectoYDibujoColision();

            [h, t, ts, zs, as, lasers, gtpose, gtposes] = simuloUnInstanteDeTiempo(ts, gtposes, lasers);

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
        save('LearningData_Folders/'+carpeta_LearningData+'/vO'+v_apren+'/Qlearning_data_vO'+v_apren+'_'+string(num_episodes)+'.mat', ...
            'total_reward_per_episode', 'total_duration_per_episode', 'distancias_siguiendo_pared', ...
            'politica_actual', 'gtposes', 'net', 'DQN_params', 'trainingOptions');
        % Guardo todos los datos mas recientes para poder seguir entrenando de
        % forma continua
        save('LearningData_Folders/'+carpeta_LearningData+'/vO'+v_apren+'/Qlearning_data_vO'+v_apren+'_mas_reciente.mat', ...
            'total_reward_per_episode', 'total_duration_per_episode', 'distancias_siguiendo_pared', ...
            'num_episodes', 'politica_actual', 'net', 'DQN_params', 'trainingOptions'); % por si acaso borro los iniciales
    end
end


% ---------------------------------------------------------------------------------------------------------------------------------------
% Funciones auxiliares
% ---------------------------------------------------------------------------------------------------------------------------------------


% Inicializo las constantes
function [paredes_mapa, Options] = inicializacionConstantes()

    % Opciones de simulacion
    Options.max_time_duration_episode = 50;
    Options.max_laser_range = 3.3; %maxima distancia tomada como valida (en m)
    Options.total_points_laser = 1080; % si quiero que coja los 3 sectores de visión
    Options.min_dist = 0.35;
    Options.max_dist = 4;


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

function [net, DQN_params, trainingOpts] = crearNuevaRedNeuronal(DQN_params)
    DQN_params.epsilon = 1.0;
    DQN_params.epsilonMin = 0.01;
    DQN_params.epsilonDecay = 0.995;
    DQN_params.learningRate = 1e-3;
    DQN_params.gamma = 0.99;

    % Tamaño de las entradas y salidas
    DQN_params.inputSize = 1080;
    DQN_params.outputSize = 3;

    % Creo la red neuronal
    net = createSimpleDQN(DQN_params.inputSize, DQN_params.outputSize);

    % Creo las opciones de entrenamiento
    trainingOpts = trainingOptions('adam', ...
        'InitialLearnRate',learningRate, ...
        'Verbose',false, ...
        'Plots','training-progress');
end

% Muestro unos mensajes al inicio de cada episodio
function mensajesIniciales()
    disp('Episodio '+string(num_episodes)+' del entrenamiento actual')
    disp('Aprendizaje para ' + string(v_apren))
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

function [h, t, ts, zs, as, lasers, gtpose, gtposes] = simuloUnInstanteDeTiempo(ts, lasers, gtposes)
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

% Creación de una red neuronal
function net = createSimpleDQN(inputSize, outputSize)
    layers = [
        featureInputLayer(inputSize, 'Normalization', 'none', 'Name', 'input')
        fullyConnectedLayer(24, 'Name', 'fc1') % Primera capa oculta con 24 neuronas
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(24, 'Name', 'fc2') % Segunda capa oculta con 24 neuronas
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(outputSize, 'Name', 'output') % Capa de salida
    ];

    net = layerGraph(layers);
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