%Ejemplos del jacobiano
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

%Espacio de trabajo del robot
 q=[3,-159*pi/180 ,-0.5 ,0.40];
 q0=[0,-150*pi/180 ,0 ,0];
 q1=[-140*pi/180,-150*pi/180,0 ,0];
 q2=[-140*pi/180,0 ,0 ,0];
 q3=[140*pi/180,0 ,0 ,0.];
 q4=[140*pi/180,150*pi/180,0 ,0];
 q5=[-70*pi/180,150*pi/180 ,0,0];
 q6=[-70*pi/180,150*pi/180 ,0.18,0];
 q7=[140*pi/180,150*pi/180 ,0.18 ,0];
 q8=[140*pi/180,0*pi/180 ,0.18 ,0];
 q9=[-140*pi/180,0*pi/180 ,0.18 ,0];
 q10=[-140*pi/180,-150*pi/180 ,0.18 ,0];
 q11=[0,-150*pi/180 ,0.18,0];
 t = [0:0.05:2]';
 qt0 = jtraj(q0, q1, t);
 qt1 = jtraj(q1, q2, t);
 qt2 = jtraj(q2, q3, t);
 qt3 = jtraj(q3, q4, t);
 qt4 = jtraj(q4, q5, t);
 qt5 = jtraj(q5, q6, t);
 qt6 = jtraj(q6, q7, t);
 qt7 = jtraj(q7, q8, t);
 qt8 = jtraj(q8, q9, t);
 qt9 = jtraj(q9, q10, t);
 qt10 = jtraj(q10, q11, t);
 qt=[qt0;qt1;qt2;qt3;qt4;qt5;qt6;qt7;qt8;qt9;qt10];
T = R.fkine(qt);
p = transl(T);


%Calculo jacobiano
coordenadas_art1=[0 0 0 0];
coord_cart2=[-0.3 0.577 0.1];
coord_cart1=[-0.3 0.575 0.1];
T1=transl(coord_cart1);
T2=transl(coord_cart2);

fprintf('Coordenadas cercanas a una singularidad: [-0.3 0.575 0.1]');
[coordenadas_art2]=inversa(R,T1,coordenadas_art1)
J1=R.jacob0(coordenadas_art2);
Jr1 =[J1(1:3,:);J1(6,:)];
condicion=cond(Jr1)
determinate=det(Jr1)
manipulabilidad=R.maniplty(coordenadas_art2,'trans')

fprintf('Coordenadas de una singularidad: [-0.3 0.577 0.1]');
[coordenadas_art3]=inversa(R,T2,coordenadas_art2)
J2=R.jacob0(coordenadas_art3);
Jr2 =[J2(1:3,:);J2(6,:)];
condicion=cond(Jr2)
determinate=det(Jr2)
manipulabilidad=R.maniplty(coordenadas_art3,'trans')


%Para plotear se debe comentar la linea 98 de la funcion animate
figure(1)
R.plot3d(coordenadas_art3,'path',pwd,'workspace',[-1 1 -1 1 -2 2],'view','top')
hold on
plot_ellipse(J2(1:3,:)*J2(1:3,:)',coord_cart2)

plot3([-0.5 0 0.5],[0.632 0.632 0.632],[0 0 0],'b', 'LineWidth', 2)
plot3([-0.5 0 0.5],[0.232 0.232 0.232],[0 0 0],'b', 'LineWidth', 2)
plot3([-0.3 -0.3 -0.3],[0.232 0.432 0.632],[0 0 0],'r--', 'LineWidth', 2)
plot3([0.3 0.3 0.3],[0.232 0.432 0.632],[0 0 0],'r--', 'LineWidth', 2)
plot3([-0.3 0 0.3],[0.632 0.632 0.632],[0 0 0],'r--', 'LineWidth', 2)
plot3([-0.3 0 0.3],[0.232 0.232 0.232],[0 0 0],'r--', 'LineWidth', 2)
plot3(p(:,1), p(:,2),p(:,3),'r','linewidth',2)
hold off
%Espacio de trabajo del robot
figure(2)
plot3(p(:,1), p(:,2),p(:,3),'r','linewidth',2)