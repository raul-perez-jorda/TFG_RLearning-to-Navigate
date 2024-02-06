num_ojos = 1;
[X,Y,Z] = meshgrid(1:1:3,1:1:3,1:1:num_ojos);
V = Qtable;

num_ojos = 2;
[Xa,Ya,Za] = meshgrid(1:1:3,1:1:3,1:1:num_ojos);
Va = interp3(X,Y,Z,V,Xa,Ya,Za,'cubic')