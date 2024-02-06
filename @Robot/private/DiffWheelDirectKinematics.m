function [x1,y1,theta1,vcm,wcm] = DiffWheelDirectKinematics(x0,y0,theta0,incangl,incangr,radiuswheell,radiuswheelr,inct,d)
% Odometry calculation through the differential wheel model.
% X0,Y0,THETA0 -> initial pose
% INCANGL,INCANGR -> increment of angular position of both wheels (radians)
% RADIUSWHEELL-R -> radii of both wheels (meters)
% INCT -> time spent in those increments of angles
% D -> distance between the two wheels, divided by 2 (meters)
% VCM -> linear velocity at the start of step (m/s)
% WCM -> angular velocity at the start of the step (rad/s)

    if (inct<0)
        error('Negative time increment.');
    end
    if (inct==0)
        x1=x0;
        y1=y0;
        theta1=theta0;
        return;
    end
    
    vr=incangr/inct*radiuswheelr;
    vl=incangl/inct*radiuswheell;
    vcm=(vr+vl)/2;
    wcm=(vr-vl)/(2*d);
    
    x1=x0+vcm*cos(theta0)*inct;
    y1=y0+vcm*sin(theta0)*inct;
    theta1=theta0+wcm*inct;
    
end
    
