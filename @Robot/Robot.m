% Class for simulation and control of a robot with two independently 
% actuated driving wheels and proprioceptive and exteroceptive sensors.
% -----------------------------------------
% (c) Juan-Antonio Fernández-Madrigal, 2022-2023
% System Engineering and Automation Dpt.
% University of Malaga (Spain)
% http://babel.isa.uma.es/jafma
% 
% This simulation has some realistic features:
%
%   - Motion of the wheels has certain dynamics: rotational velocities
%   follow a first order time response.
%
%   - Range finder has longitudinal noise in the measurements. For advanced
%   users (see defineRobot() method), even an angular noise and a beam-like
%   thickness may be simulated.
%
%   - For advanced users (see defineRobot()), the wheels may have different
%   radii.
%
% The functions of this library allow the Matlab programmer to command
% basic actions in a simulated mobile robot and reading its 
% sensors. NOTE: Only the methods in the "Actuators and sensors" section
% below are available in a real robot; the rest of them provide the exact
% state of the system, only available in the simulated robot.
%
% The methods are (please use "help Robot.<method>" to get more details):
%
%   --------- Creation / deletion of the object ------------
%
%	Robot -> create the simulation object.
%
%   --------- Definition of the simulation environment ------------
%
%   defineRobot -> define the physical features of the robot
%
%   getRobotDef -> return the current robot definition
%
%   defineWalls -> define the walls (obstacles) in the environment
%
%   getWalls -> return the current definition of walls
%
%   pointInFreeSpace -> return whether a point is within the free space
%
%   --------- General state of the robot ------------
%
%   changePose -> change the ground-truth pose of the robot at once
%
%   getPose -> get the current ground-truth pose of the robot
%
%	robot2univMatrix -> homogeneous transform matrices between the robot system
%						and the universal system.
%
%   getAngWheel -> get the current angle of a wheel, without the
%                  discretization of the encoders.
%
%   collision -> calculate if the robot is colliding with the walls.
%
%   setUserData -> set user data in the robot object.
%
%   getUserData -> return the current user data.
%
%   --------- Actuators and sensors ------------
%
%   setMotor -> change the power of one motor
%
%   getMotor -> read the current power in one motor
%
%   setSpeeds -> set whole robot speeds
%
%   getSpeeds -> get whole robot speeds
%
%   getLimitSpeeds -> read the max/min speeds of the robot
%
%   readEncoder -> read one encoder
%
%   readRange -> read the egocentric data of the rangefinder
%
%   calcRangefinderBeams -> calculate the universal reference data for all
%                           beams of the rangefinder.
%
%   --------- Simulation ------------
%
%   getSimTime -> return the current simulation time
%
%   simulate -> simulate the robot for a given duration
%
%   getFigure -> get the handler of the figure where the robot is being
%                drawn, or an empty figure if no figure is being used.
%
%   redraw -> redraw the simulator figure.
%
%   changeDrawEnv -> change the area of the environment to draw to a fixed
%                    one.
%
%   changeDrawEnvMov -> change the area of the environment to draw to one
%                    that moves with the robot.
%
%   setRangeDrawing -> enable or disable the drawing of the range sensor data.
%
%
% VERSION HISTORY
%
% [01/11/23] v0.0.2
%
%   -Fixed a bug in the wall collision detection for the rangefinder.
%
% [30/03/22] v0.0.1
%
%	-Created for the first time by reusing the Matlab Lego simulator LMML.

classdef Robot < handle
    % It is a handle for being modified when passed to a function; copies
    % will refer to the same object, but you can create any number of
    % objects.
    
    properties (Constant, Access = public)
    
        VER = '0.0.2';
        
    end
    
    properties (Access = private)

        feats; % physical parameters of the robot. See the defineRobot() 
               % method

        robshape; % (calculated) convex hull in the form of a list of 2D 
                  % segments, in the robot reference system.
        rangeangles; % (calculated) a vector with the angle w.r.t. the 
                     % rangefinder system for the center of each 
                     % rangefinder beam.

        gt_pose; % current robot pose on the plane: x, y, theta
        t; % current simulated time
        angs; % rotational position of wheels: 1-> left, 2-> right, radians
        alphas; % rotational speeds of wheels: idem, in rad/s
        motors; % power on motors: idem, in -100..100, integer     
        encs; % encoder values: idem, in degrees, not discretized
        lastrange; % vector in meters with all beams, NaN if no measurement
                   % no discretized.
          
        walls; % matrix with as many rows as walls, each one with four 
               % values: x0,y0,x1,y1
        environ; % struct with these fields:
                 %  .mode -> 0 for fixed, 1 for moving with the robot
                 %      .minx, .miny, .maxx, .maxy -> parameters of fixed
                 %      .radiusx, .radiusy -> parameters of moving
        disp; % figure handler, or empty if not drawing anything
        drawrange; % drawing mode for the rangefinder: 1 for drawing ending
                   % points, 2 for drawings beams, 0 for not drawing.

        userdata; % user data.

        figdata; % graphic elements. %vicente
    end
    
    
    methods (Access = public)
        
        % ----- Constructor

        function obj = Robot(di)
        % Construct a new simulator. 
        %
		% 	s = Robot(di);
        %
        % DI must be 1 to open a figure for showing the robot or 0 to work 
        % without figure.
        % This constructor fills the robot parameters with default ones.
        
            % default physical features (see the defineRobot() method)
            obj.defineRobot(struct('d',0.23,... % wheel distance
                               'r',[0.02,0.02],... 
                               'hull',[0.354,0.354,0.177],... 
                               'beamorig',[-0.15;0;0], ... 
                               'wrange',deg2rad(270),...
                               'orange',deg2rad(-270/2),...
                               'nrange',1080,...
                               'maxrange',4,...
                               'semiangle_range',deg2rad(0),...
                               'sigmas_range',[0.001,0]));
                           
            % initial configuration
            obj.t = 0;
            obj.gt_pose = [0,0,0].';
            obj.encs = [0,0];      
            obj.angs = [0,0];
            obj.alphas = [0,0];
            obj.motors = [0,0];
            obj.lastrange = nan(1,obj.feats.nrange);
            
            obj.walls = [];
                                             
            obj.environ = struct('mode',1,...
                                 'radiusx',0.5,...
                                 'radiusy',0.5);

            obj.userdata = [];

            if (di == 0)
				obj.disp = [];
            else
                obj.setupFig();
            end

        end

        % ----- Destructor
        
        function delete(obj)

            if obj.checkFigure()
                close(obj.disp);
            else
                fprintf('No simulation figure to delete.\n');
            end
            fprintf('Deleted Robot object.\n');
            
        end
        
        % ----- Public methods

        function clearFigure(obj)
        % Clear all dynamic data from figure: robot, range,
        % paths?, collisions?
        % 
		% 	s.clearFigure();
            
            if obj.checkFigure()
                delete(obj.figdata.robot);
                delete(obj.figdata.range);

                obj.figdata.robot = [];
                obj.figdata.range = [];
                fprintf('Dynamic objects deleted.\n');
            else
                fprintf('No figure to clear.\n');
            end
        end

        function defineRobot(obj,features)
        % Define the physical features of the robot. By default, the robot
        % has some built-in values in these features.
        % 
		% 	s.defineRobot(features);
		%
        % FEATURES must be a struct with these fields:
        %   .d -> distance between both wheels, in meters
        %   .r -> vector with the 2 radii of the wheels (1-> left, 2-> 
        %         right), in meters
        %   .hull -> vector with 3 values defining a rectangular convex 
        %            hull for the robot body: 1-> longitudinal size of the 
        %            robot body (along the direction of motion); 2-> 
        %            transversal size of the robot body; 3-> distance
        %            from the center of movement to the front of the body 
        %            longitudinally. All in meters.
        %   .beamorig -> origin of all beams of the range finder in 
        %            coordinates of the robot reference system.
        %   .wrange -> range in radians of the rangefinder (e.g., 2*pi is a
        %            rangefinder covering the entire circumference around
        %            the robot, while pi may represent a frontal
        %            rangefinder).
        %   .orange -> angle in radians of the fisrt range beam measured
        %            from the x axis of the rangefinder reference system,
        %            counterclockwise.
        %   .nrange -> number of beams covering the rangefinder angle
        %            range.
        %   .maxrange -> meters indicating the maximum range of a beam.
        %   .semiangle_range -> half of the angle of one beam,
        %            where any obstacle will be detected without problems
        %            (in rads).
        %   .sigmas_range -> a vector of two sigmas for the gaussian noises 
        %            in the longitudinal (1) and angular (2) parameters of 
        %            each range beam, in meters.
        
            if ~isfield(features,'d')
                error('Wheel distance must be in field "d"');
            end
            Robot.checkIsPositive(features.d,'.d field');
      
            if ~isfield(features,'r')
                error('Wheel radii must be in field "r"');
            end
            Robot.checkIsVectorOfN(features.r,2,'.r field');
            Robot.checkIsPositive(features.r(1),'.r(1)');
            Robot.checkIsPositive(features.r(2),'.r(2)');

            if ~isfield(features,'hull')
                error('Robot convex hull must be in field "hull"');
            end
            Robot.checkIsVectorOfN(features.hull,3,'.hull field');
            Robot.checkIsPositive(features.hull(1),'.hull(1)');
            Robot.checkIsPositive(features.hull(2),'.hull(2)');
            Robot.checkIsPositiveOrZero(features.hull(3),'.hull(3)');
            
            if ~isfield(features,'beamorig')
                error('Origin of rangefinder beams must be in field "beamorig"');
            end
            Robot.checkIsVectorOfN(features.beamorig,3,'.beamorig field');
            
            if ~isfield(features,'wrange')
                error('Width of the range finder must be in field "wrange"');
            end
            Robot.checkIsPositive(features.wrange,'.wrange field');

            if ~isfield(features,'orange')
                error('Angle of first beam must be in field "orange"');
            end
            Robot.checkIsNumScalar(features.orange,'.orange field');

            if ~isfield(features,'nrange')
                error('Number of range finder beams must be in field "nrange"');
            end
            Robot.checkIsInteger(features.nrange,'.nrange field');

            if ~isfield(features,'maxrange')
                error('Max. range of a beam must be in field "maxrange"');
            end
            Robot.checkIsPositive(features.maxrange,'.maxrange field');

            if ~isfield(features,'semiangle_range')
                error('The range beam half-angle must be in field "semiangle_range"');
            end
            Robot.checkIsPositive(features.d,'.semiangle_range field');
            
            if ~isfield(features,'sigmas_range')
                error('Range noises must be in field "sigmas_range"');
            end
            Robot.checkIsVectorOfN(features.sigmas_range,2,'.sigmas_range field');

            obj.feats = features;
            obj.updateRobShape();
            obj.updateRangeAngles();
            obj.lastrange = nan(1,obj.feats.nrange);
            
            fprintf('Robot physical features changed ok.\n');
        
        end
        
        function features = getRobotDef(obj)
        % Get the current definition of robot physical features
        %
		% 	features = s.getRobotDef();
		%
        % FEATURES will be as in the "defineRobot" method.
        
            features = obj.feats;
        
        end
        
        function defineWalls(obj,ws)
        % Change the current wall definition by the one in WS.
        %
        % WS is a matrix with as many rows as walls and 4 columns with the
        % (x,y) coordinates of the ends of each wall.
        
            if (~isempty(ws))
                [~,nc] = size(ws);
                if (nc ~= 4)
                    error('WS must have 4 columns');
                end
            end
            obj.walls = ws;

        	if obj.checkFigure()
                obj.drawWalls();
	        end

            fprintf('Walls definition changed ok.\n');

        end

        function ws = getWalls(obj)
        % Get the current definition of walls
        % 
		% 	ws = s.getWalls();
        %
        % WS will be as in the "defineWalls" method.
        
            ws = obj.walls;
        
        end
        
        function changePose(obj,p)
        % Change the ground-truth pose of the robot to P. It also reset 
        % the gyro.
        % 
		% 	s.changePose(p);
        %
        % P must be a column vector with 3 real numbers: x,y,theta, being
        % (x,y) in meters and theta in radians.
        
            Robot.checkIsVectorOfN(p,3,'The pose');
            [r,~] = size(p);
            if (r ~= 3)
                error('Poses must be column vectors.');
            end
            obj.gt_pose = p;
            obj.lastrange = nan(1,obj.feats.nrange);

        	if obj.checkFigure()
                obj.drawRobot();
            end

            fprintf('Real robot pose changed ok.\n');
        
        end
        
        function p = getPose(obj)
        % Get the current ground-truth pose of the robot.
        % 
		% 	p = s.getPose();
        %
        % P is a column vector as in the "changePose" method.
        
            p = obj.gt_pose;
        
        end
        
        function a = getAngWheel(obj,wheel)
        % Get the exact angle in radians of the position of wheel WHEEL,
        % without the discretization of the encoder.
        % 
		% 	a = s.getAngWheel(wheel);
        %
        % WHEEL must be 'L' or 'R'.
        
            which = Robot.checkMotor(wheel);
            a = obj.angs(which);
        
        end
        
        function [c,i] = collision(obj)
        % Calculate if the robot is colliding with the walls using the
        % convex hull of the robot.
        % 
		% 	[c,i] = s.collision();
        %
        % C will be 1 for a collision or 0 for no collision. In the former
        % case, I will be the index of the first wall that produces such a
        % collision.
        
            c = 0;
            i = NaN;
            if (isempty(obj.walls))
                return;
            end
            
            [~,rTu] = obj.robot2univMatrix();
            
            [nw,~] = size(obj.walls);
            hulllims = obj.robotHullLimits();
            for (f = 1:nw)
                p0r = rTu * ([obj.walls(f,1:2) 0 1].');
                p1r = rTu * ([obj.walls(f,3:4) 0 1].');
                p0in = obj.pointInHull(p0r,hulllims); 
                p1in = obj.pointInHull(p1r,hulllims);
                if p0in || p1in
                    c = 1;
                    i = f;
                    break;
                else % wall may cross the hull
                    x0 = p0r(1);
                    y0 = p0r(2);
                    x1 = p1r(1);
                    y1 = p1r(2);
                    
                    vx = x1-x0;
                    vy = y1-y0;
                    if (vy ~= 0)
                        l1 = (hulllims(2) - y0)/vy;
                        xi1 = x0 + l1 * vx;
                        l2 = (hulllims(4) - y0)/vy;
                        xi2 = x0 + l2 * vx;
                        if ((l1 >= 0) && (l1 <= 1) && (xi1 >= hulllims(1)) && (xi1 <= hulllims(3))) || ...
                           ((l2 >= 0) && (l2 <= 1) && (xi2 >= hulllims(1)) && (xi2 <= hulllims(3)))
                            c = 1;
                            i = f;
                            break;
                        end
                    end
                    if (vx ~= 0)
                        l1 = (hulllims(1) - x0)/vx;
                        yi1 = y0 + l1 * vy;
                        l2 = (hulllims(3) - x0)/vx;
                        yi2 = y0 + l2 * vy;
                        if ((l1 >= 0) && (l1 <= 1) && (yi1 >= hulllims(2)) && (yi1 <= hulllims(4))) || ...
                           ((l2 >= 0) && (l2 <= 1) && (yi2 >= hulllims(2)) && (yi2 <= hulllims(4)))
                            c = 1;
                            i = f;
                            break;
                        end
                    end                                       
                end
            end
        
        end

        function infree = pointInFreeSpace(obj,p,withrobot)
        % Detect whether point P is in the free space region of the
        % simulated environment. NOTE: if the point is outside the entire
        % environment, it will be considered as in free space only if there
        % are no walls; otherwise it will be considered in non-free space.
        %
        %   infree = s.pointInFreeSpace(p,withrobot);
        %
        % P is a vector (x,y) in meters, in the universal frame.
        % WITHROBOT must be 1 to include the body of the robot as non-free
        % space or 0 to ignore the robot.
        
            if withrobot
                % check whether the point is inside the robot
                [~,rTu] = obj.robot2univMatrix;
                q = rTu * [p(1) ; p(2); 0 ; 1]; % pass P to coords of the robot
                hulllims = obj.robotHullLimits();
                if obj.pointInHull(q,hulllims) % point within robot shape
                    infree = 0;
                    return;
                end
            end

            % check if the point is within walls
            if isempty(obj.walls)
                infree = 1;
                return;
            end

            % check if there is an odd number of horizontal intersections
            % to the right
            nw = size(obj.walls,1);
            count = 0;
            for (f = 1:nw)
                w = obj.walls(f,:);
                if (w(2) == w(4)) % horizontal wall
                    if (p(2) == w(2)) % point on the wall
                        infree = 0;
                        return;
                    end % otherwise, ignore this wall
                else
                    % pass wall to point frame
                    wx1 = w(1) - p(1);
                    wy1 = w(2) - p(2);
                    wx2 = w(3) - p(1);
                    wy2 = w(4) - p(2);
                    if (sign(wy1) ~= sign(wy2))
                        % check
                        if (wx1 == wx2) % vertical wall
                            if (sign(wx1) == 1)
                                count = count + 1;
                            end
                        else
                            xc = wx1 + (wx2-wx1)/(wy2-wy1)*(-wy1);
                            if (sign(xc) == 1)
                                count = count + 1;
                            end
                        end
                    end % otherwise, no intersection with the wall
                end
            end
            if (mod(count,2) == 1) % odd intersections == in free space
                infree = 1;
            else
                infree = 0;
            end

        end

		function setMotor(obj,motor,power)        
		% Set the power of one motor (it is the percentage of the desired
        % steady-state wheel rotation speed).
        % Note that this does not animate the robot drawing or simulate any
        % time increment in the system.
        % 
		% 	s.setMotor(motor,power);
		%
		% MOTOR is the motor, either 'L' or 'R'
		% POWER is the power, an integer within -100 and 100	

            which = Robot.checkMotor(motor);
            Robot.checkPower(power);
            obj.motors(which) = power;

        end

        function p = getMotor(obj,motor)
		% Get last power set to one motor. 
        % 
		% 	p = s.getMotor(motor);
		%
		% MOTOR is the motor, either 'L' or 'R'
		% P is the power, an integer within -100 and 100	
           
            which = Robot.checkMotor(motor);
            p = obj.motors(which);

        end

        function setSpeeds(obj,v,w)
        % Set motor powers to target the given linear and angular robot
        % speeds. 
        % Note that this does not animate the robot drawing or simulate any
        % time increment in the system.
        %
        %   s.setSpeeds(v,w)
        %
        % V is the desired linear speed (m/s) and W the desired angular
        % speed (rad/s). The calculated motor powers will be clamped to 
        % [-100,100].

            vR = v + obj.feats.d * w / 2;
            vL = v - obj.feats.d * w / 2;
            alphaR = vR / obj.feats.r(2);
            alphaL = vL / obj.feats.r(1);
            mps = robotmotionparms();
            pL = alphaL / mps.vftestl * 100;
            pR = alphaR / mps.vftestr * 100;
            if (pL > 100)
                pL = 100;
            elseif (pL < -100)
                pL = -100;
            end
            if (pR > 100)
                pR = 100;
            elseif (pR < -100)
                pR = -100;
            end
            obj.setMotor('L',pL);
            obj.setMotor('R',pR);

        end

        function [v,w] = getSpeeds(obj)
        % Get linear and angular robot speeds corresponding to the latest
        % power commands to the motors.
        %
        %   [v,w] = s.setSpeeds()
        %
        % V is the desired linear speed (m/s) and W the desired angular
        % speed (rad/s). 

            mps = robotmotionparms();
            pL = obj.getMotor('L')/100;
            pR = obj.getMotor('R')/100;
            alphaL = mps.vftestl * pL;
            alphaR = mps.vftestr * pR;
            v= (alphaR * obj.feats.r(2) + alphaL * obj.feats.r(1))/2;
            w= (alphaR * obj.feats.r(2) - alphaL * obj.feats.r(1))/obj.feats.d;

        end
        
        function [minv,maxv,minw,maxw] = getLimitSpeeds(obj)
        % Return the minimum and maximum linear (V) and angular (W) speeds
        % of the whole robot, in m/s and rad/s respectively.

            mps = robotmotionparms();
            % maximum and minimum linear speeds given with max/min wheel
            % rotation speeds
            maxv = (mps.vftestr * obj.feats.r(2) + mps.vftestl * obj.feats.r(1)) / 2;
            minv = -maxv;
            % maximum and minimum angular speeds given with max/min wheel
            % rotation speed discrepancy
            maxw = (mps.vftestr * obj.feats.r(2) - (-mps.vftestl * obj.feats.r(1))) / obj.feats.d;
            minw = -maxw;

        end

			
		function enc = readEncoder(obj,motor)
		% Read the encoder of one motor, as an integer value in degrees.
        % 
		% 	enc = s.readEncoder(motor);
		%
		% MOTOR is the motor, either 'L' or 'R'
        
            which = Robot.checkMotor(motor);
            enc = round(obj.encs(which));
		
		end
		
		function [zs,as] = readRange(obj)
		% Return the last rangefinder that was simulated. 
		% If none has been simulated yet, or the sensor has not
		% detected any obstacle, return the maximum range.
        % 
		% 	[zs,as] = s.readRange();
		%
        % ZS is a vector with as many values as rangefinder beams, filled
        % counterclockwise, in meters. If some beam has no measurement, it 
        % will be the maximum range.
        % AS is a vector of the same length as ZS containing the angles of
        % the center of the beams w.r.t. the rangefinder reference frame
        % (see defineRobot()).
        % This DOES NOT simulate a new sensor reading; just takes the last
        % one simulated with the simulate() method.

            zs = ones(1,obj.feats.nrange) * obj.feats.maxrange;
%             for (f = 1:obj.feats.nrange)
%                 if ~isnan(obj.lastrange(f))
%                     %zs(f) = round(obj.lastrange(f));
%                     zs(f) = obj.lastrange(f); % raul
%                 end
%             end
            ind = ~isnan(obj.lastrange); % vicente
            zs(ind) = obj.lastrange(ind);
            as = obj.rangeangles;

		end   

        function [posorigbeamu,beamsu,beamthetasu] = calcRangefinderBeams(obj)
        % Calculate the current rangefinder data and positions w.r.t. the
        % universal frame.
        %
        %   [posorigbeamu,beamsu,beamthetasu] = s.calcRangefinderBeams();
        %
        % POSORIGBEAMU is a column vector with the [X;Y;Z;1] homogeneous 
        % coordinates of the origin of the rangefinder beams.
        % BEAMSU is a cell with as many elements as rangefinder beams, each
        % one containing the homogeneous transform from each beam reference
        % system to the universal system (the beam system has the X axis
        % pointing along the beam, the Z axis pointing upwards and the
        % origin at the same origin as all beams).
        % BEMTHETASU is a column vector with as many elements as beams
        % containing the angle of each beam w.r.t. the universal frame X
        % axis (counterclockwise).

            [uTr,~] = obj.robot2univMatrix();
            posorigbeamu = uTr * [obj.feats.beamorig;1]; % rangefinder origin in U
            incangrange = obj.feats.wrange / (obj.feats.nrange - 1);
            angrange = obj.feats.orange;
            beamthetasu = zeros(obj.feats.nrange,1); % universal angle w.r.t. X for each beam
            beamsu = cell(1,obj.feats.nrange);
            for (f = 1:obj.feats.nrange)
                rRs = [cos(angrange) cos(pi/2+angrange) 0 ; ...
                       sin(angrange) sin(pi/2+angrange) 0 ; ...
                       0 0 0];
                beamsu{f} = [rRs obj.feats.beamorig ; 0 0 0 1];
                posbeamu = uTr * beamsu{f} * [1;0;0;1];
                beamthetasu(f) = atan2(posbeamu(2)-posorigbeamu(2),posbeamu(1)-posorigbeamu(1)); % in -pi...+pi
                angrange = angrange + incangrange;
            end
                            
        end
                       
        function [uTr,rTu] = robot2univMatrix(obj)
        % Calculate the homogeneous transform matrix to pass coordinates from 
        % the robot system to the universal one (uTr) and its inverse (rTu).
        %
        %	[uTr,rTu] = s.robot2univMatrix();
            
            c = cos(obj.gt_pose(3));
            s = sin(obj.gt_pose(3));
            uTr = [ c -s 0 obj.gt_pose(1) ; ...
                    s c  0 obj.gt_pose(2) ; ...
                    0 0  1 0 ; ...
                    0 0  0 1 ]; % uTr
            rTu = [ uTr(1:3,1:3).' -(uTr(1:3,1:3).')*uTr(1:3,4) ; ...
                    0 0 0 1];
                
        end

        function st = getSimTime(obj)
        % Return the current simulation time in seconds.
        %
        %	st = s.getSimTime();
        
            st = obj.t;
        
        end

        function simulate(obj,inct)
        % Simulate the system during the next INCT seconds with 100
        % intermediate steps. Afterwards, redraw the scenario. 
        % It also simulates a new reading of the rangefinder after that 
        % increment of time.
        %
        %	s.simulate(inct);
        % 
        % Note that this method does not check collisions; use the
        % collision() method after calling this one.
        %
        % INCT is the time to simulate from now.
        
            Robot.checkIsPositive(inct,'INCT');
            
            oldstate = struct('gtpose',obj.gt_pose.',... 
                                'encl',obj.encs(1),... 
                                'encr',obj.encs(2),... 
                                'wl',obj.alphas(1),...
                                'wr',obj.alphas(2),...
                                'al',obj.angs(1),...
                                'ar',obj.angs(1),...
                                't',obj.t...
                                );

            % simulate motion for 100 steps within INCT
            newstate = simulationstep(obj.motors(1),obj.motors(2),...
                                      inct,oldstate,...
                                      obj.feats.d,...
                                      obj.feats.r(1),obj.feats.r(2));
            obj.gt_pose = newstate.gtpose.';
            obj.encs = [newstate.encl newstate.encr];
            obj.alphas = [newstate.wl newstate.wr];
            obj.angs = [newstate.al newstate.ar];
            obj.t = newstate.t;

            % simulate the rangefinder after motion
            [posorigbeamu,~,beamsu] = obj.calcRangefinderBeams();
            for (f = 1:obj.feats.nrange)
                obj.lastrange(f) = simulationrangebeam(posorigbeamu(1),posorigbeamu(2),beamsu(f),...
                                    obj.walls,...
                                    obj.feats.semiangle_range,...
                                    obj.feats.sigmas_range(1),...
                                    obj.feats.sigmas_range(2),...
                                    obj.feats.maxrange);
                if (obj.lastrange(f) >= obj.feats.maxrange - 1e-6)
                    obj.lastrange(f) = NaN;
                end
            end

            obj.redraw();

        end

        function h = getFigure(obj)
        % Return the handler of the figure where the robot is being drawn
        % or an empty vector if no figure is being used.
        %
        %	h = s.getFigure();
        
            if (isempty(obj.disp))
                h = [];
            else
                h = obj.disp;
            end
        
        end

        function redraw(obj)
        % Redraw the simulator figure.
        %
        %	s.redraw();
        
        	if (~isempty(obj.disp))
	            obj.drawAll();
	        end
        
        end
        
        function changeDrawEnv(obj,area,equalaxes)
        % Change the viewport of the simulator to one fixed on the plane.
        %
        %	s.changeDrawEnv(area,equalaxes);
        %
        % AREA is a vector with minx, miny, maxx, maxy
        % If EQUALAXES == 1, adjust the area to include the desired one but
        % having aspect ratio == 1.
        
            Robot.checkIsVectorOfN(area,4,'area');
            incx = area(3) - area(1);
            incy = area(4) - area(2);
            if (incx <= 0) || (incy <= 0)
                error('Invalid viewport area');
            end
            obj.environ = struct('mode',0,...
                                 'minx',area(1),...
                                 'miny',area(2),...
                                 'maxx',area(3),...
                                 'maxy',area(4));
            if (equalaxes)
                obj.adjustAspect();
            end
        
        end

        function changeDrawEnvMove(obj,radii,equalaxes)
        % Change the viewport of the simulator to one that moves along the
        % robot.
        %
        %	s.changeDrawEnvMove(radii,equalaxes);
        %
        % RADII is the width of the viewport in the x axis (1) and in the y
        % axis (2) that will be shown.
        % If EQUALAXES == 1, adjust the area to include the desired one but
        % having aspect ratio == 1.

            Robot.checkIsVectorOfN(radii,2,'radii');
            if (radii(1) <= 0) || (radii(2) <= 0)
                error('Invalid viewport area');
            end
            obj.environ = struct('mode',1,...
                                 'radiusx',radii(1),...
                                 'radiusy',radii(2));
            if equalaxes
                obj.adjustAspect();
            end
        
        end

        function setRangeDrawing(obj,ed)
        % Enable (ED == 1) or disable (ED == 0) the drawings of the
        % rangefinder data. If ED == 2, enable it and also draws the beams.
        %
        %	s.setRangeDrawing(ed);
        
            Robot.checkIsNumScalar(ed,'ed');
            obj.drawrange = ed;
        
        end

        function setUserData(obj,ud)
        % Change current user data by UD.
        %
        %   s.setUserData(ud);

            obj.userdata = ud;

        end

        function ud = getUserData(obj)
        % Get the current user data.
        %
        %   ud = s.getUserData();

            ud = obj.userdata;

        end

        function c = getCurrentChar(obj)

            if isempty(obj.disp)
                c = [];
            else
                c = obj.figdata.currentChar;
            end

        end

    end % --- end public methods
    

    methods (Static, Access = private)

        function unimplemented()
        % produce an unimplemented error
            error('Unimplemented operation');
        end

        function checkNonEmpty(v,txt)
            if (isempty(v))
                error('%s must be non-empty',txt);
            end
        end
        
        function checkIsNumScalar(v,txt)
            if (~isscalar(v))
                error('%s must be a scalar',txt)
            end
            if (ischar(v))
                error('%s must be a numeric scalar',txt);
            end
        end
        
        function checkIsVectorOfN(v,n,txt)
            if length(v) ~= n
                error('%s must have %d elements',txt,n);
            end
        end
        
        function checkIsHT(t,txt)
            [r,c] = size(t);
            if (r ~= 4) || (c ~= 4)
                error('%s is not a 4x4 matrix',txt);
            end
        end
        
        function checkIsPositive(v,txt)
            Robot.checkNonEmpty(v,txt);
            Robot.checkIsNumScalar(v,txt);
            if (v <= 0)
                error('%s must be positive',txt)
            end
        end

        function checkIsPositiveOrZero(v,txt)
            Robot.checkNonEmpty(v,txt);
            Robot.checkIsNumScalar(v,txt);
            if (v < 0)
                error('%s must be positive or zero',txt)
            end
        end
        
        function checkIsInteger(v,txt)
            Robot.checkIsPositive(v,txt);
            if (floor(v) ~= v)
                error('%s must be an integer number');
            end
        end
        
        function checkPower(v)
            Robot.checkNonEmpty(v,'Power ');
            Robot.checkIsNumScalar(v,'Power ');
%             if (mod(v,1) ~= 0)
%                 error('Power must be an integer');
%             end
            if (v < -100) || (v > 100)
                error('Power must be in [-100,+100]');
            end
        end
        
        function which = checkMotor(v)
            if v == 'L'
                which = 1;
            elseif v == 'R'
                which = 2;
            else
                error('Invalid motor name');
            end            
        end

        function in = pointInHull(p,hull) 
        % Calculate if the point P is within the rectangle HULL.
        %
        %   in = Robot.pointInHull(p,hull);
        %
        % P is a vector with 2 elements: x and y.
        % HULL is a vector of 4 elements with minx, miny, maxx, maxy

            if (p(1) > hull(1)) && (p(1) < hull(3)) && ...
               (p(2) > hull(2)) && (p(2) < hull(4))
                in = 1;
            else
                in = 0;
            end
            
        end
        
    end
    
    methods (Access = private)

        function figexists = checkFigure(obj)
        % If the robot is being displayed and there is a figure for it
        % opened, return 1; otherwise return 0

            if (isempty(obj.disp))
            	figexists = false;
            else
	            figexists = (findobj(obj.disp,'type','figure') == obj.disp);
            end
            
        end

        function adjustAspect(obj)

            switch obj.environ.mode
                case 0
                    incx = obj.environ.maxx - obj.environ.minx;
                    incy = obj.environ.maxy - obj.environ.miny;
                case 1
                    incx = obj.environ.radiusx;
                    incy = obj.environ.radiusy;
                otherwise
                    error('Invalid viewport mode');
            end
            switch obj.environ.mode
                case 0
                    ar = incy/incx;
                    if ar > 1
                        xneeded = incy-incx;
                        obj.environ.minx = obj.environ.minx - xneeded/2;
                        obj.environ.maxx = obj.environ.maxx + xneeded/2;
                    else
                        yneeded = incx-incy;
                        obj.environ.miny = obj.environ.miny - yneeded/2;
                        obj.environ.maxy = obj.environ.maxy + yneeded/2;
                    end
                case 1
                    m = max([obj.environ.radiusx,obj.environ.radiusy]);
                    obj.environ.radiusx = m;
                    obj.environ.radiusy = m;
            end
            
        end
        
        function updateRobShape(obj)
            
            lx = obj.feats.hull(1); % longitudinal length
            ly = obj.feats.hull(2); % transversal length
            lf = obj.feats.hull(3); % displacement of center
            lxmlf = lx - lf;
            lyd2 = ly/2;
            lo = min(obj.feats.hull(1:2))/2;
            obj.robshape = [-lxmlf,lyd2; ...
                            lf,lyd2;...
                            lf,0;...
                            lf+lo,0;...
                            lf,0;...
                            lf,-lyd2;...
                            -lxmlf,-lyd2;...
                            -lxmlf,lyd2];            

        end

        function updateRangeAngles(obj)

            obj.rangeangles = linspace(obj.feats.orange, ...
                                       obj.feats.orange + obj.feats.wrange, ...
                                       obj.feats.nrange);

        end

        function hull = robotHullLimits(obj)

            hull = [ -(obj.feats.hull(1)-obj.feats.hull(3)), ... % minx
                     -obj.feats.hull(2)/2, ... % miny
                     obj.feats.hull(3), ... % maxx
                     obj.feats.hull(2)/2 ]; % maxy

        end

        function onKeyPressHandle(obj,fig,event)
        
            obj.figdata.currentChar = event.Character;
        
        end
        
        function onKeyReleaseHandle(obj,fig,event)
        
            obj.figdata.currentChar = [];
        
        end

        function setupFig(obj) % vicente
        % figure setup

            if isempty(obj.disp)
                obj.disp = figure( ...
                    KeyPressFcn=@obj.onKeyPressHandle, ...
                    KeyReleaseFcn=@obj.onKeyReleaseHandle ...
                );
            else
                figure(obj.disp);
                clf;
            end

            % setup axes
            hold on;
            axis equal;
            grid on;

            % plot the axes lines'
            obj.figdata.axes.x = ...
                plot([0 1],[0 0],'r-');
            obj.figdata.axes.y = ...
                plot([0 0],[0 1],'y-');

            obj.figdata.walls = [];
            obj.figdata.robot = [];
            obj.figdata.range = [];

            obj.figdata.currentChar = [];

	        obj.drawrange = 1;

        end

        function drawWalls(obj)
        % Draw in figure H the walls of the environment.

			if ~obj.checkFigure()
				error('Cannot draw the walls without figure');
			end
%             if ~obj.checkFigure()
%                 fprintf('Re-creating simulation figure.\n');
%                 obj.setupFig();
%             end

            % bring figure to foreground
            figure(obj.disp);

            % draw walls
            delete(obj.figdata.walls);
            for f = 1 : length(obj.walls)
                obj.figdata.walls(f) = ...
                    plot([obj.walls(f,1) obj.walls(f,3)],...
                            [obj.walls(f,2) obj.walls(f,4)],...
                            'k-','LineWidth',3);
            end

        end

        function drawRobot(obj)
        % Draw in figure H the robot body at the given pose with the given color
        % MODE is the shape of the robot:
        %   'point' -> a small circle with orientation
        %   'shape' -> a shape with the vertices in SHAPE (that must include the
        %              first vertex repeated at the end in order to close it)

			if ~obj.checkFigure()
				error('Cannot draw robot without figure');
			end
%             if ~obj.checkFigure()
%                 fprintf('Re-creating simulation figure.\n');
%                 obj.setupFig();
%             end

            % bring figure to foreground
            figure(obj.disp);

            % calculations for drawing
            [uTr,~] = obj.robot2univMatrix();
            rangeorig = uTr * [obj.feats.beamorig(:); 1];

            colorrob = 'bw'; % border and fill

            shape = obj.robshape;
            numpts = length(shape);
            xs = zeros(1,numpts);
            ys = zeros(1,numpts);
            for f = 1 : numpts
                p = uTr * [shape(f,:).';0;1];
                xs(f) = p(1);
                ys(f) = p(2);
            end
            if isempty(obj.figdata.robot)
                obj.figdata.robot(1) =...
                    plot(uTr(1,4),uTr(2,4),[colorrob(1) '.'],'MarkerSize',8);
                obj.figdata.robot(2) =...
                    plot(xs,ys,[colorrob(1) '-']);
                obj.figdata.robot(3) =...
                    plot(rangeorig(1),rangeorig(2),[colorrob(1) 'o']);
            else
                set(obj.figdata.robot(1),...
                    'XData',uTr(1,4),'YData',uTr(2,4));
                set(obj.figdata.robot(2),...
                    'XData',xs,'YData',ys);
                set(obj.figdata.robot(3),...
                    'XData',rangeorig(1),'YData',rangeorig(2));
            end

            % draw range data
            if obj.drawrange

                % calculations for drawing
                [posorigbeamu,beamstransfu,~] = obj.calcRangefinderBeams();
                plu = nan(obj.feats.nrange,2);
                for f = 1 : obj.feats.nrange
                    z = obj.lastrange(f);
                    if (~isnan(z))
                        bs = uTr * beamstransfu{f} * [z;0;0;1];
                        plu(f,:) = [bs(1) bs(2)];
                    end
                end
    
                if all(isnan(plu)) % nothing to draw
                    return;
                end

                if isempty(obj.figdata.range)
                    obj.figdata.range(1) =...
                        plot(posorigbeamu(1),posorigbeamu(2),[colorrob(1) '.']);
                    obj.figdata.range(2) =...
                        plot(plu(:,1),plu(:,2),'g.');
                else
                    set(obj.figdata.range(1),... % fail with nan values
                        'XData',posorigbeamu(1),'YData',posorigbeamu(2));
                    set(obj.figdata.range(2),... % fail with nan values
                        'XData',plu(:,1),'YData',plu(:,2));
                end
            end        

        end

        function drawAll(obj)

			if ~obj.checkFigure()
				error('Cannot draw without figure');
			end
%             if ~obj.checkFigure()
%                 fprintf('Re-creating simulation figure.\n');
%                 obj.setupFig();
%             end

            % change viewport
            switch (obj.environ.mode)
                case 0 % fixed
                    xlim([obj.environ.minx obj.environ.maxx]);
                    ylim([obj.environ.miny obj.environ.maxy]);
                case 1 % moving
                    xlim(obj.gt_pose(1) + obj.environ.radiusx * [-1,1]);
                    ylim(obj.gt_pose(2) + obj.environ.radiusy * [-1,1]);
                otherwise
                    error('Invalid viewport mode');
            end

            % draw robot (and range)
            obj.drawRobot();

            % print robot params
            pL = obj.getMotor('L');
            pR = obj.getMotor('R');
            [v,w] = obj.getSpeeds();
            title(sprintf('[RoSiMat v%s]  t=%.3fs pL=%.2f pR=%.2f v=%.3f w=%.3f',...
                          Robot.VER,obj.t,...
                          pL,pR,v,w));

            drawnow; % allows dispaching events
        end
                    
    end

end
