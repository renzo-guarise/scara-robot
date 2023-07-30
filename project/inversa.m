%Funcion inversa
%Realiza la cinematica inversa del robot ABB IRB910 SC de manera analitica.
%Entrada:
%-Robot: objeto robot SerialLink en estudio.
%-T: matriz de transfomacion homogenea.
%-coord_artant: vector articular semilla.
%Salida:
%-coord_art1: coordenadas articulares. Devuelve la mejor solucion de
%cinematica inversa, siendo la mejor la mas cercana al vector semilla
%pasado.
%-bool: Devuelve un bool que es True cuando las coordenadas cartesianas se
%encuentran dentro del espacio de trabajo y se cumple con los limites
%articulares si no devuelve falso.

function[coord_art1,bool]=inversa(Robot,T,coord_artant)
    
    %Referenciamos nuestra coordenadas con respecto a la base del robot
    T3=Robot.base.double; T3(3,4)=0;
    coord_cart=invHomog(T3)*[transl(T);1];
    coord_cart=coord_cart(1:3)';
    %Realizamos los calculos para obtener q1
    beta=atan2(coord_cart(2),coord_cart(1));
    l=(coord_cart(2)^2+coord_cart(1)^2)^0.5;
    aux_cos=((Robot.links(2).a)^2-(Robot.links(1).a)^2-(l^2))/(-2*(Robot.links(1).a)*l);
    alfa1=real(acos(aux_cos));
    alfa2=-alfa1;
    q11=beta+alfa1;
    q12=beta+alfa2;
    %Verificamos que los q1 obtenidos esten dentro del limite articular
    [bool_veri1,coord]=verificacion([q11,0,0,0],Robot);
    [bool_veri2,coord1]=verificacion([q12,0,0,0],Robot);
    if (bool_veri1==0)
        q12=coord1(1)-Robot.offset(1);
        q11=q12;
    else
        q11=coord(1)-Robot.offset(1);
    end
    
    if (bool_veri2==0)
        q11=coord(1)-Robot.offset(1);
        q12=q11;
    else
        q12=coord1(1)-Robot.offset(1);
    end
    %Verificamos si la posicion cartesiana esta dentro del espacio de
    %trabajo
    if isreal(alfa1)
        bool=1;
    else
        bool=0;
    end
    if (bool_veri1==0 && bool_veri2==0)
        bool=0;
    end
    %Calculo de q2 para cada q1
    T1 = Robot.links(1).A(q11).double;
    p21=invHomog(T1)*[coord_cart 1]';
    coord_aux(1)=atan2(p21(2),p21(1));
    [bool_ver1,coord]=verificacion([0,coord_aux(1),0,0],Robot);
    coord_aux(1)=coord(2)-Robot.offset(2);
    T1 =Robot.links(1).A(q12).double;
    p21=invHomog(T1)*[coord_cart 1]';
    coord_aux(2)=atan2(p21(2),p21(1));
    [bool_ver2,coord]=verificacion([0,coord_aux(2),0,0],Robot);
    coord_aux(2)=coord(2)-Robot.offset(2);
    %Verificamos que los q2 se encuentren dentro de los limites articulares
    if (bool_ver1==0 && bool_ver2==0)
        bool=0;
    end
    %Calculamos la mejor solucion
    if((((q11-coord_artant(1))^2+(coord_aux(1)-coord_artant(2))^2)^0.5)>((q12-coord_artant(2))^2+(coord_aux(2)-coord_artant(2))^2)^0.5)
        coord_art1(1)=q12;
        coord_art1(2)=coord_aux(2);
    else
        coord_art1(1)=q11;
        coord_art1(2)=coord_aux(1);
    end

    %Calculamos q3
    q=[0,0,0,0];
    aux=transl(Robot.fkine(q));
    Z_lim(1)=aux(3)-Robot.offset(3);
    q=[0,0,0.18,0];
    aux=transl(Robot.fkine(q));
    Z_lim(2)=aux(3)-Robot.offset(3);
    
    %Verificamos que q3 se encuentre dentro de los limites articulares
    if ((coord_cart(3)>Z_lim(1))) 
         bool=0;
         coord_cart(3)=Z_lim(1);
    else if(coord_cart(3)<Z_lim(2))
             bool=0;
             coord_cart(3)=Z_lim(2);
         end
    end
    
    coord_art1(3)=Z_lim(1)-coord_cart(3);
    coord_art1(4)=0;
    
    %Calculamos q4
    T_aux=Robot.fkine(coord_art1);
    T_aux2=T_aux.double\T;
    aux1=tr2rpy(T_aux2);
    aux11=aux1(3);
    
    if aux11<0
        aux12=aux11+2*pi; 
    else
        aux12=aux11-2*pi; 
    end
    if abs(coord_artant(4)-aux11)>abs(coord_artant(4)-aux12)
        coord_art1(4)=aux12;
    else
        coord_art1(4)=aux11;
    end
    
    
    
end
function iT = invHomog(T)
iT = eye(4);
iT(1:3, 1:3) = T(1:3, 1:3)';
iT(1:3, 4) = - iT(1:3, 1:3) * T(1:3, 4);
end