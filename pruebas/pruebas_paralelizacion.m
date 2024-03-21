clear; close all;
delete(gcp('nocreate'))

Qtable = zeros(3)
Visitas = zeros(3)

parpool(3)
parfor i=1:30 
    x = randi(1,3); y = randi(1,3);
    Qtable(x,y) = randi(1,10);
    Visitas(x,y) = Visitas(x,y)-1;
end

sum(Visitas)