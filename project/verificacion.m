%Verifica si las coordenadas articulares estan dentro de los limites del
%robot
%Devuelve:
%-bool:Devuelve 1 si las coordenadas son correctas y 0 si son incorrectas.
%-coord_art: En caso de ser correcta las coordenadas pasadas como
%argumentos devuelve las misma. Si estas son
%incorrectas devuelve el valor mas cercano a las coordenadas deseadas.
%-n: Devuelve el numero de articulacion donde se traduce el error. Si es
%que se produce algun error.
%-T: Matriz homogenea correspondientes a las coordenadas articulares.
function [bool,coord_art,n,T]=verificacion(pos_art,robot)

    bool=1;n=0;
    for i=1:length(pos_art)

        if ~(robot.qlim(i,1)<=pos_art(i) && pos_art(i) <=  robot.qlim(i,2))
           n=i; 
           bool=0;
           if(abs(robot.qlim(i,1)-pos_art(i))<abs(robot.qlim(i,2)-pos_art(i)))
               coord_art(i)=robot.qlim(i,1);
           else
               coord_art(i)=robot.qlim(i,2);
           end
        else
        coord_art(i)=pos_art(i);
        end
        
    end
    T=robot.fkine(coord_art);
end