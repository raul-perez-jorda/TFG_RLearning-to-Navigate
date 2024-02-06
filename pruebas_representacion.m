clear all; close all;

% Opciones del Q-Learning
QLearningOpt.max_time_duration_episode = 30; %seconds
QLearningOpt.total_actions = 3;
QLearningOpt.discount_factor = 0.99;
QLearningOpt.learning_rate = 0.1;
QLearningOpt.exploration_factor = 0.1;

% Opciones de simulacion
Options.max_laser_range = 3.3; %maxima distancia tomada como valida (en m)
Options.number_of_laser_regions = 3; % para calcular el ransac de la pared derecha
Options.total_points_laser = 1080; % si quiero que coja los 3 sectores de visión
Options.total_points_laser = 2*1080/3; % si guiero que solo coja el sector derecho y frontal

% Defino las caracteristicas de mi struct de estados discretizados
stateArrayOpt.num_ojos = 8;
stateArrayOpt.num_distancias = 3;

stateArrayOpt.total_states = stateArrayOpt.num_distancias^stateArrayOpt.num_ojos; %: numero de estados posibles
stateArrayOpt.num_actions = 3;

stateArrayOpt.min_dist = 0.4;
stateArrayOpt.max_dist = 3;


% Mensaje bienvenida
s = Robot(1);
fprintf('Robot definition: \n');
robotdef = s.getRobotDef()
fprintf('Robot max/min speeds: \n');
[minv,maxv,minw,maxw] = s.getLimitSpeeds()

fprintf('Robot physical measurements with noise: \n');
%measuredradii = robotdef.r + normrnd(0,0.003/2,1,2);
%measureddistw = robotdef.d + normrnd(0,0.0075/2);

% Definicion postura inicial y entorno (paredes)
s.changePose([3,3,0].'); % POR DETERMINAR
        
s.defineWalls([0 2 2 2; ...
    2 2 2 0; ...
    3 0 3 1.5; ...
    3 1.5 4 2; ...
    4 2 4.5 3; ...
    4.5 3 6 3; ...
    1 6 2 5; ...
    2 5 4 5; ...
    4 5 4 4; ...
    4 4 6 4]);


s.changeDrawEnv([0 0 6 6],1); % Defino el tamaño del 'mapa de simulación'
s.setRangeDrawing(0);

TL = 1; % tiempo para hacer el lado recto
TG = 1; % cannot be too low for not producing motosr powers below 15
vlin = 15;
vang = 10;
steptime = 0.032;

% Inicializar variables
odomposes = [];
gtposes = [];
sonars = [];
encs = [];
motors = [];
ts = [];
st = 0;
lastst = s.getSimTime();
i_st = 1; % indice de estados para secuencia de movimientos
oldt = lastst;
oldencs = [s.readEncoder('L') s.readEncoder('R')]; % degs
oldp = s.getPose();
finish = 0;
ind = 1;
t0 = tic;

vec_ptos_validos = boolean([]);

alphas_deg1 = []; alphas_deg2 = []; alphas_deg3 = [];
ds1 = []; ds2 = []; ds3 = [];
quals1 = []; quals2 = []; quals3 = [];
ms1=[]; ms2=[]; ms3=[];
ns1=[]; ns2=[]; ns3=[];

set(gcf,'CurrentCharacter','@'); %lo iniciamos como un valor de poco uso

x_r = oldp(1); y_r = oldp(2); % posicion inicial del robot

% Bucle principal
%while (~finish)

    s.simulate(steptime);
    [c,iw] = s.collision();
    if c % si hay colision
        figure(s.getFigure());
        ws = s.getWalls();
        hold on;
        plot(ws(iw,[1 3]),ws(iw,[2 4]),'r-','LineWidth',2);
        error('Collision with wall #%d',iw);
    end 
    ts = [ts s.getSimTime()];
    [zs,as] = s.readRange();
    sonars = [sonars ; zs];
    encs = [encs ; s.readEncoder('L') s.readEncoder('R')];
    motors = [motors ; s.getMotor('L') s.getMotor('R')];
    h = s.getFigure();
    if ~isempty(h)
        figure(h);
        hold on;
        if (~isempty(gtposes))
            plot(gtposes(1,:),gtposes(2,:),'b-');
            %plot(odomposes(:,1),odomposes(:,2),'r-');
        end
    end
   
    t = s.getSimTime();

    gtpose = s.getPose();
    gtposes = [gtposes gtpose];
    
    % next step
    oldt = s.getSimTime();
    oldencs = [s.readEncoder('L') s.readEncoder('R')];    
    %oldp = p;
    ind = ind + 1;

    % Dibujar el las paredes a traves del sonar y el algoritmo RANSAC
        %Obtener los puntos en coordenadas cartesianas
        ptos_cartes = [zs'.*cos(as'), zs'.*sin(as')]+3; %puntos del sonar en coordenadas cartesianas (respecto del robot)

        figure
        plot(ptos_cartes(:,1), ptos_cartes(:,2), '.m');
        axis equal;

        points_region = Options.total_points_laser/Options.number_of_laser_regions; %points laser divided # regions
        ptos_cartes1 = ptos_cartes(1:360,:); %puntos de la parte derecha del robot
        ptos_cartes2 = ptos_cartes(361:720,:); %puntos de la parte frontal del robot
        ptos_cartes3 = ptos_cartes(721:1080,:); %puntos de la parte izquierda del robot

        % Limpiar puntos del laser con distancia 20
        vec_ptos_validos = logical([]);
        for i=1:1:length(zs)
            if(zs(i) > Options.max_laser_range)
                vec_ptos_validos(i,1) = false;
            else
                vec_ptos_validos(i,1) = true;
            end
        end

        ptos_cartes1 = ptos_cartes1(vec_ptos_validos(1:360), :);
        ptos_cartes2 = ptos_cartes2(vec_ptos_validos(361:720), :);
        ptos_cartes3 = ptos_cartes3(vec_ptos_validos(721:1080), :);

        [ransac_x1, ransac_y1, m, n, alpha, d, qual] = RANSAC_y_triplete(ptos_cartes1, 2, 0.01);
            alphas_deg1 = [alphas_deg1; rad2deg(alpha)];
            ds1 = [ds1,d];
            quals1 = [quals1; qual];
            ms1 = [ms1; m];
            ns1 = [ns1; n];
        [ransac_x2, ransac_y2, m, n, alpha, d, qual] = RANSAC_y_triplete(ptos_cartes2, 2, 0.01);
            alphas_deg2 = [alphas_deg2; rad2deg(alpha)];
            ds2 = [ds2,d];
            quals2 = [quals2; qual];
            ms2 = [ms2; m];
            ns2 = [ns2; n];
        % [ransac_x3, ransac_y3, m, n, alpha, d, qual] = RANSAC_y_triplete(ptos_cartes3, 2, 0.01);
        %     alphas_deg3 = [alphas_deg3; rad2deg(alpha)];
        %     ds3 = [ds3,d];
        %     quals3 = [quals3; qual];
        %     ms3 = [ms3; m];
        %     ns3 = [ns3; n];

        [stateArrayCont, stateArrayDisc] = get_stateArrays(zs, as, stateArrayOpt, Options);
        current_state = traductor_stateArray2Qindex(stateArrayDisc,stateArrayOpt);
        representa_stateArray(stateArrayDisc, stateArrayCont,stateArrayOpt, gtpose, ransac_x1, ransac_y1)

        % figure(2);
        % subplot(2,1,1)
        hold on
        plot(ptos_cartes1(:,1),ptos_cartes1(:,2),'k.', ...
            ptos_cartes2(:,1),ptos_cartes2(:,2),'k.', ...
            ptos_cartes3(:,1),ptos_cartes3(:,2),'k.')
        % axis equal
        % subplot(2,1,2)
        % plot(ransac_x1,ransac_y1,'g-', ...
        %     ransac_x2,ransac_y2,'r-', ...
        %     ransac_x3,ransac_y3,'b-', ...
        %     0, 0, 'k*') % paredes alrededor a partir de laser
        axis equal
        xlim([0 6]); ylim([0 6])
        hold off;
        
%end
toc(t0)