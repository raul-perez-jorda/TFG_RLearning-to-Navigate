function z = simulationrangebeam(xbeamu,ybeamu,thetabeamu,map,semianglebeam,sigmalong,sigmaperp,maxz)
% Simulates the reading of a range beam (Z, in meters) if the beam start is at 
% (xbeamu,ybeamu) pointing in angle thetabeamu, all w.r.t. the universal 
% frame, given an obstacle MAP that contains the walls.
%
% SEMIANGLEBEAM is the half-angle of the sonar beam, in rads.
% SIGMALONG is the sigma parameter of a gaussian centered at the ground
% truth collision of the sonar with the obstacle (0 for no noise
% simulation)
% SIGMAPERP is the sigma parameter of a gaussian centered in the angle of 
% the ray of the sonar (0 for no noise in simulation)
% MAXZ is the maximum distance returned by the sonar in any case (or in
% case it does not collide with anything). In meters.

    z=maxz;
    if (isempty(map))
        return;
    end
    
    % ray casting
   
    [nobsts,~] = size(map);
    for (f=1:nobsts) % for each wall
        xp1 = map(f,1);
        yp1 = map(f,2);
        xp2 = map(f,3);
        yp2 = map(f,4);
        if (sigmaperp > 0)
            thetadatum = thetabeamu+normrnd(0,sigmaperp);
        else
            thetadatum = thetabeamu;
        end
        if (semianglebeam <= 0) % simulate a linear beam
            [thereiscutmM,posM,lambdamM]=raysegmentcut(xbeamu,ybeamu,thetadatum,xp1,yp1,xp2,yp2);
            thereiscutmL = 0;
            thereiscutmR = 0;
            if posM ~= 0
                thereiscutmM = 0;
            end
        else % simulate a thick beam
            [thereiscutmL,posL,lambdamL]=raysegmentcut(xbeamu,ybeamu,thetadatum - semianglebeam,xp1,yp1,xp2,yp2);
            [thereiscutmM,posM,lambdamM]=raysegmentcut(xbeamu,ybeamu,thetadatum,xp1,yp1,xp2,yp2);
            [thereiscutmR,posR,lambdamR]=raysegmentcut(xbeamu,ybeamu,thetadatum + semianglebeam,xp1,yp1,xp2,yp2);
            if posL ~= 0
                thereiscutmL = 0;
            end
            if posM ~= 0
                thereiscutmM = 0;
            end
            if posR ~= 0
                thereiscutmR = 0;
            end            
        end
        if ~thereiscutmL
            lambdamL = Inf;
        end
        if ~thereiscutmM
            lambdamM = Inf;
        end
        if ~thereiscutmR
            lambdamR = Inf;
        end
        zm = min([lambdamL lambdamM lambdamR]);
        if (~isinf(zm))
            if (sigmalong>0)
                zm=zm+normrnd(0,sigmalong);
            end
        else
            zm = maxz;
        end

        if (zm<z)
            z=zm;
        end
    end        
       
end

function [thereiscut,position,lambda]=raysegmentcut(xray,yray,thetaray,xp1,yp1,xp2,yp2)
% Calculate the cut point of the given ray with the line that supports the
% given segment. Return in THEREISCUT 1 if there is a cut with the segment, 
% 0 if there is no cut (other parameters not filled), in LAMBDA the 
% parametric value of the cut point within the ray, and in POSITION 0 if 
% the cut is within the segment, -1 if the cut is before (xp1,yp1), i.e., 
% at the opposite side of (xp2,yp2), and 1 if the cut is beyond (xp2,yp2)

    thereiscut=0;
    position=0;
    lambda=0;
    
    ux=cos(thetaray);
    uy=sin(thetaray);
    wx=xp2-xp1;
    wy=yp2-yp1;
    % vector equation to solve: u * lambda = (p1 - ray) + w * beta
    % u is the vector of the ray, unitary.
    % lambda is the distance to the collision point, if any, along u.
    % p1 is the first point of the wall and ray the origin point of the
    % ray, thus p1-ray is the first point of the wall if the origin of
    % coordinates is the origin of the ray.
    % w is the wall vector, from first to last point.
    % beta is the position along the wall (maybe < 0) of the collision
    % point.
    A=[ux -wx ; uy -wy];
    if (rank(A)<2)
        return; % no solution to equation: no collision possible
    end
    E=[xp1-xray ; yp1-yray];
    S=A\E;
    lambda = S(1);
    beta = S(2);
    if (lambda<0.0)
        return; % collision at the back of the ray: no collision
    end
    thereiscut=1; % collision exists, but maybe out of the wall segment
    if (beta<0.0)
        position=-1; % collision where there is no wall: before first point
    elseif (beta>1.0)
        position=1; % collision where there is no wall: after last point
    else
        position=0; % collision within the wall
    end
    
end

