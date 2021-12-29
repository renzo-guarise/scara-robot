%Simulacion sobre la tarea que va a realizar nuestro robot
%Para plotear se debe comentar la linea 98 de la funcion animate

clc, clear, close all


dh = [
    0.000  0.1992  0.400  0.000 0;
    0.000  0.0585  0.250  pi 0;
    0.000  0.000   0 pi   1;
    0.000  0.000   0  0 0];

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
%Puntos caracteristicos de la aplicacion
pmedio=[0 0.432 0.220];
pliminferior=[0,0.232,0];
psuperior=[0 0.602 0];
pplastico=[0.5 -0.1,0.220];
pmetal=[-0.5 -0.1,0.220];
pvidrio=[-0.4,-0.3,0.220];
ppapeles=[0.4,-0.3,0.220];

%Eleccion aleatoria de puntos sobre la cinta
s=rng();
xr=-0.3+0.6*rand(20,1);
yr=0.232+0.37*rand(20,1);
zr=eye(20,1)*0.05;
cor=[xr,yr,zr];
cor_art(1,:)=inversa(R,transl(pvidrio),[0 0 0 0]);
j=1;
ps=[pmetal;pplastico;pvidrio;ppapeles];
dt=0.05;
t=0:dt:1;
%Interpolacion con jtraj y ctraj
%%
for i=2:5:100
    a=datasample(ps,1,1);
    cor_art(i,:)=inversa(R,transl(pmedio),cor_art(i-1,:));
    cor_art0=jtraj(cor_art(i-1,:),cor_art(i,:),t);
    cor_art(i+1,:)=inversa(R,transl([0,cor(j,2),0.220]),cor_art(i,:));
    cor_art1=jtraj(cor_art(i,:),cor_art(i+1,:),0:0.05:0.5);
    cor_art(i+2,:)=inversa(R,transl(cor(j,:)),cor_art(i+1,:));
    cor_cart=ctraj(transl([0,cor(j,2),0.220]),transl(cor(j,:)),length(0:0.05:0.5));
    cor_art2(1,:)=inversa(R,cor_cart(:,:,1),cor_art(i+1,:));
    for x=2:length(cor_cart(1,1,:))
        cor_art2(x,:)=inversa(R,cor_cart(:,:,x),cor_art2(x-1,:));
    end
    cor_art(i+3,:)=[cor_art(i+2,1:2),0.01,cor_art(i+2,4)];
    cor_art3=jtraj(cor_art(i+2,:),cor_art(i+3,:),0:0.05:0.5);
    cor_art(i+4,:)=inversa(R,transl(a),cor_art(i+3,:));
    cor_art4=jtraj(cor_art(i+3,:),cor_art(i+4,:),t);
    cc(j,:,:)=[cor_art0;cor_art1;cor_art2;cor_art3;cor_art4];
    j=j+1;
end

cc2(:,:)=[cc(1,:,:),cc(2,:,:),cc(3,:,:),cc(4,:,:),cc(5,:,:),cc(6,:,:),cc(7,:,:),cc(8,:,:),cc(9,:,:),cc(10,:,:),cc(11,:,:),cc(12,:,:),cc(13,:,:),cc(14,:,:),cc(15,:,:),cc(16,:,:),cc(17,:,:),cc(18,:,:),cc(19,:,:),cc(20,:,:)];
figure(1)
plot3([-0.5 0 0.5],[0.602 0.602 0.602],[0 0 0],'b', 'LineWidth', 2)
hold on
plot3([-0.5 0 0.5],[0.232 0.232 0.232],[0 0 0],'b', 'LineWidth', 2)
plot_circle([pplastico(1:2),0],0.1,'y', 'LineWidth', 2)
plot_circle([pmetal(1:2),0],0.1,'r', 'LineWidth', 2)
plot_circle([pvidrio(1:2),0],0.1,'g', 'LineWidth', 2)
plot_circle([ppapeles(1:2),0],0.1,'b', 'LineWidth', 2)
plot3([-0.3 -0.3 -0.3],[0.232 0.432 0.602],[0 0 0],'r--', 'LineWidth', 2)
plot3([0.3 0.3 0.3],[0.232 0.432 0.602],[0 0 0],'r--', 'LineWidth', 2)
plot3([-0.3 0 0.3],[0.602 0.602 0.602],[0 0 0],'r--', 'LineWidth', 2)
plot3([-0.3 0 0.3],[0.232 0.232 0.232],[0 0 0],'r--', 'LineWidth', 2)
plot3(xr,yr,zr,'bx')

%Para plotear se debe comentar la linea 98 de la funcion animate
R.plot3d(cc2,'path',pwd,'workspace',[-1 1 -1 1 -1 1],'trail', {'r', 'LineWidth', 1},'delay',0.01,'view','top')
hold off