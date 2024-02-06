function simulationdrawrobot(h,uTr,theta,colorrob,mode,shape)
% Draw in figure H the robot body at the given pose with the given color
% MODE is the shape of the robot:
%   'point' -> a small circle with orientation
%   'shape' -> a shape with the vertices in SHAPE (that must include the
%              first vertex repeated at the end in order to close it)

    x = uTr(1,4);
    y = uTr(2,4);
    
    figure(h);
    hold on;
    if (strcmp(mode,'point')==1)
        orlen=0.02;
        plot(x,y,strcat(colorrob(1),'o'));
        plot([x x+orlen*cos(theta)],...
             [y y+orlen*sin(theta)],...
            sprintf('%c-',colorrob(1)));
        return;
    end
    
    if (strcmp(mode,'shape')==1)
        [numpts,~]=size(shape);
        xs = zeros(1,numpts);
        ys = zeros(1,numpts);
        for (f=1:numpts)
            p = uTr * [shape(f,:).';0;1];
            xs(f) = p(1);
            ys(f) = p(2);
        end
        plot(xs,ys,sprintf('%c-',colorrob(1)));
        %plot([xs xs(1)],[ys ys(1)],sprintf('%c-',colorrob(1)));
        %fill(xs,ys,colorrob(2));
        plot(x,y,sprintf('%c.',colorrob(1)),'MarkerSize',8);
        return;
    end
    
    error('Drawing mode unknown.');

end
