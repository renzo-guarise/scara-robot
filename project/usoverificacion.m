%Pequeño ejemplo sobre el uso de la funcion verificacion

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

fprintf('Salida de la funcion verificacion:')
[bool,coord_articulares,n,T]=verificacion([pi/2,pi,0,1],R)
R.plot(coord_articulares,'jvec','base','tiles','workspace',[-1 1 -1 1 -0.75 1.25])


