%%Programa que muestra el uso de la funcion inversa
%Para plotear se debe comentar la linea 98 de la funcion animate

clc, clear, close all


dh = [
    0.000  0.1992  0.400  0.000 0;
    0.000  0.0585  0.250  pi    0;
    0.000  0.000   0      pi    1;
    0.000  0   0      0     0];

R = SerialLink(dh,'name','IRB910 SC');

% Limites
R.qlim(1,1:2) = [-140,  140]*pi/180;
R.qlim(2,1:2) = [-150,  150]*pi/180;
R.qlim(4,1:2) = [-400,  400]*pi/180;
R.qlim(3,1:2) = [0, 0.18];

extremoFinal = -0.037;
longGripper = 0.213;

%Base, tool y offset
R.tool = transl(0, 0, extremoFinal-longGripper);
R.base = transl(0,0,longGripper)*trotz(pi/2);
R.offset = [ 0 0 0 0];

%Punto cartesiano al que se quiere ir
fprintf('Matriz homogenea de la posicion a la que se quiere ir:')
T=transl([0,-0.5,0.1])
coord_artant=[0 0 0 0];

fprintf('Resultados funcion inversa:')
[coord_art1,bool]=inversa(R,T,coord_artant)

%Para plotear se debe comentar la linea 98 de la funcion animate
R.plot3d(coord_art1,'path',pwd,'workspace',[-1 1 -1 1 -1 1],'delay',0.01,'view','top')
hold on
trplot(T)

