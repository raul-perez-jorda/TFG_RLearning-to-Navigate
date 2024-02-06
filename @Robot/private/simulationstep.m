function newstate = simulationstep(potl,potr,dur,oldstate,...
                                   d,rl,rr)
% simulate one step of dynamics of the robot and return the new state
% NOTE: the ending time must be taken from the corresponding field of
% newstate, not calculated by the calling code
% 
% POTL,POTR are the power given to the wheels in this step, in [-100,100]
% DUR is the time in seconds that is simulated
% OLDSTATE is a struct that holds all information of the previous steps of
% simulation. It has the following fields:
%   -'gtpose' -> ground truth pose (x,y,theta) at end of step (m & rad)
%   -'encl' -> value of encoder of left wheel (degrees) at end of step
%   -'encr' -> the same for right wheel
%   -'wl' -> turning velocity of left wheel (rad/sec) at end of step
%   -'wr' -> the same for right wheel
%   -'al' -> angle turned by left wheel (rad) at end of step
%   -'ar' -> the same for left wheel
%   -'wend' -> angular velocity of the robot at the end of step (rad/s)
%   -'vend' -> linear veloticy of the robot at the end of step (m/s)
%   -'t' -> absolute time when the rest of fields where measured
% D is the distance between both wheels, in meters.
% RL, RR are the radii of the wheels, in meters.

    % motor parameters
    mps = robotmotionparms();

    % simulate dynamics of wheels
    poteffl=potl;
    poteffr=potr;
    if (mps.deadzonemotors>0)
        if (abs(potl)<=mps.deadzonemotors)
            poteffl=0;
        end
        if (abs(potr)<=mps.deadzonemotors)
            poteffr=0;
        end
    end
    resolsim=dur/100;
    [vsl,tsl]=simularotacionrueda(poteffl,oldstate.wl,dur,resolsim,...
                                  mps.vftestl,mps.xtestl,mps.txtestl,mps.ctestl);
    [vsr,tsr]=simularotacionrueda(poteffr,oldstate.wr,dur,resolsim,...
                                  mps.vftestr,mps.xtestr,mps.txtestr,mps.ctestr);
    endwl=vsl(end);
    endwr=vsr(end);
    endt=tsl(end)+oldstate.t;
    endangl=oldstate.al+trapz(tsl,vsl);
    endangr=oldstate.ar+trapz(tsr,vsr);

    % simulate encoders
    codifl=simulacodificadorrueda(vsl,tsl,mps.resolenc);                        
    codifr=simulacodificadorrueda(vsr,tsr,mps.resolenc);
    endencl=codifl(end)+oldstate.encl;
    endencr=codifr(end)+oldstate.encr;
                              
    % calculate new pose
    x0=oldstate.gtpose(1);
    y0=oldstate.gtpose(2);
    theta0=oldstate.gtpose(3);
    [x1,y1,theta1,vcm,wcm]=DiffWheelDirectKinematics(x0,y0,theta0,...
                              endangl-oldstate.al,endangr-oldstate.ar,...
                              rl,rr,...
                              endt-oldstate.t,...
                              d/2);
                              
    % generate new state
    newstate=struct('gtpose',[x1,y1,theta1],... 
                    'encl',endencl,... 
                    'encr',endencr,... 
                    'wl',endwl,...
                    'wr',endwr,...
                    'al',endangl,...
                    'ar',endangr,...
                    'vend',vcm,...
                    'wend',wcm,...
                    't',endt...
                    );
end

function [vs,ts]=simularotacionrueda(pot,v0,t1,resol,...
                                     vftest,xtest,txtest,ctest)
% Simula la rotacion de una rueda gobernada por un primer orden
% a la que se le da la potencia (amplitud del escalon de entrada) POT
% La rueda tiene una velocidad inicial angular V0, y la simulación debe 
% durar desde el tiempo 0 hasta el T1.
% RESOL es el tiempo entre pasos de simulacion, en segundos.
% Este simulador simula perfectamente el primer orden en todos sus pasos,
% es decir, el valor de salida devuelto para cada tiempo es el mismo
% independientemente del numero de pasos de simulacion que ejecute antes.
% Eso quiere decir que la salida para el tiempo 0 y la salida para el
% tiempo T1 son las velocidades perfectas, reales, que tendría la rueda en
% esos momentos, aunque no simulemos ningún otro momento intermedio.
%
% vftest rad/seg vel. maxima en el test
% xtest porcentaje de vftest mirado en el test  
% txtest milisegundos en llegar a X% de vf
% ctest potencia dada en el test
%
%
% VS son las velocidades angulares en cada punto de resolucion de la
% simulacion
% TS son los tiempos en que se ha calculado la rotacion, desde 0

    [a,b]=calculaparametrosprimerorden(ctest,vftest,txtest,xtest);
    [vs,ts]=primerorden(a,b,pot,t1,v0,resol);
    
end

function [a,b]=calculaparametrosprimerorden(c,yf,tx,x)
% Calcula los dos parametros de un primer orden ay'+by=cx dada una entrada
% escalón de amplitud c, sabiendo que el sistema en esas condiciones y
% con c.i.=0, tarda tx segundos en llegar a (1-x)*yf, y que tiene como salida
% final yf

    if (yf==0)
        fprintf('Yf no puede ser 0!\n');
    end
    y0=0;
    b=c/yf;
    if (yf>y0)
        a=-tx*b/log((1-x)*yf/(yf-y0));
    else
        a=-tx*b/log((1-x)*yf/(y0-yf));
    end
    
end

function [ys,ts]=primerorden(a,b,c,t1,y0,resol)
% Simula el valor que tendria un primer orden de ganancia K y polo p 
% despues de T1 segundos si partiera de la condicion inicial Y0
% en pasos de RESOL segundos, devolviendo los puntos simulados
%
% La funcion de este primer orden es: 
%  y(t) = c/b*(1-exp(-b/a*t))+y(0)*exp(-b/a*t)
%
%  donde el sistema es a*dy+b*y=c*u y la entrada es u=escalon unitario
%
%  El sistema tiene un polo en 0, un cero y otro polo
%
%  No puede pasar que a==0 (no sería de primer orden) o b==0 (sería
%  inestable), aunque c sí puede ser 0

    ts=[0:resol:t1];
    if (ts(end)<t1)
        ts(end+1)=t1;
    end
    numps=length(ts);
    ys=zeros(1,numps);
    if (b==0)|(a==0)
        fprintf('Error: b o a son cero!\n');
        return;
    end
    ys=c/b*(1-exp(-b/a*ts))+y0*exp(-b/a*ts);
    
end

function codifs=simulacodificadorrueda(perfilvelang,tsvelang,cuentaporgrado)
                                             
% Simula el sensor codificador de las ruedas del robot: dado un vector de velocidades
% angulares PERFILVELANG (rad/s) y los tiempos en que se midieron (TSVELANG, seg), y
% suponiendo que en el tiempo inicial el codificador marcaba 0 de cuenta y que el encoder
% cuenta CUENTAPORGRADO por cada grado de giro, devuelve en CODIFS un
% vector con los valores del encoder

    numms=length(tsvelang);
    codifs=[];
    if (cuentaporgrado<=0)
        fprintf('ERROR: el codificador debe contar más de 0 unidades por grado de giro\n');
        return;
    end
    if (numms~=length(perfilvelang))
        fprintf('ERROR: longitudes de vector de velocidades y de tiempos distintas\n');
        return;
    end

    codifs=zeros(1,numms);
    for (f=2:numms)
        codifs(f)=trapz(tsvelang(1:f),perfilvelang(1:f));
    end
    codifs=floor(codifs*180/pi*cuentaporgrado);

end
    
