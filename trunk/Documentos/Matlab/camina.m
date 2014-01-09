
%%Prueba Camina

clear all;
clc;

%Parametros de trayectoria
T = 3;
div = 30;
beta = 0.5;
landa = 0.04;
dh = 0.02;
x_offset = 0;
y_offset = 0.11;
z_offset = -0.08;

desfasaje = 0.5;
alfa = -pi/2;

vel_apoyo = landa/beta;
vel_transferencia = (2*dh+landa)/(1-beta);
t_tramoABC = (dh/vel_transferencia)+beta;
t_tramoABCD = ((dh+landa)/vel_transferencia)+beta;

t_trayectoria=[];
x = [];
y = [];
z = [];
q1 = [];
q2 = [];
q3 = [];
ang_b = [];
ang_tg = [];
i = 0;

delta_t=0;
        
while i<=4*div
    
    i=i+1;
    delta_t = delta_t + 1/div;
    t_trayectoria(i) = delta_t + desfasaje;
     t_aux = mod(delta_t + desfasaje,1);
     Tramo(i) = t_aux ;
    
    if((0<=t_aux) && (t_aux<beta))
        t_Tramo = t_aux;
        s=0;
        
        [x_s0(i),y_s0(i),z_s0(i)]=trayectoria(landa, dh, vel_apoyo, vel_transferencia, t_Tramo, s);
        
        x(i) = x_offset+x_s0(i)*cos(alfa)-y_s0(i)*sin(alfa);
        y(i) = y_offset+x_s0(i)*sin(alfa)+y_s0(i)*cos(alfa);
        z(i) = z_offset+z_s0(i);
        
        [q1(i),q2(i),q3(i),ang_b(i),ang_tg(i)]=cinver(x(i),y(i),z(i),i);
    end;
    
    if ((beta<=t_aux) && (t_aux<t_tramoABC)) 
        t_Tramo=t_aux-beta;
        s=1;
        
        [x_s0(i),y_s0(i),z_s0(i)]=trayectoria(landa, dh, vel_apoyo, vel_transferencia, t_Tramo, s);
        
        x(i) = x_offset+x_s0(i)*cos(alfa)-y_s0(i)*sin(alfa);
        y(i) = y_offset+x_s0(i)*sin(alfa)+y_s0(i)*cos(alfa);
        z(i) = z_offset+z_s0(i);
        
        [q1(i),q2(i),q3(i),ang_b(i),ang_tg(i)]=cinver(x(i),y(i),z(i),i);
    end;
    
    if ((t_tramoABC<=t_aux) && (t_aux<t_tramoABCD))
        t_Tramo=t_aux-t_tramoABC;
        s=2;
        
        [x_s0(i),y_s0(i),z_s0(i)]=trayectoria(landa, dh, vel_apoyo, vel_transferencia, t_Tramo, s);
        
        x(i) = x_offset+x_s0(i)*cos(alfa)-y_s0(i)*sin(alfa);
        y(i) = y_offset+x_s0(i)*sin(alfa)+y_s0(i)*cos(alfa);
        z(i) = z_offset+z_s0(i);
        
        [q1(i),q2(i),q3(i),ang_b(i),ang_tg(i)]=cinver(x(i),y(i),z(i),i);
    end;
    
    if(t_tramoABCD<=t_aux) && (t_aux<1)    
        t_Tramo = t_aux-t_tramoABCD;
        s=3;
        
        [x_s0(i),y_s0(i),z_s0(i)]=trayectoria(landa, dh, vel_apoyo, vel_transferencia, t_Tramo, s);
        
        x(i) = x_offset+x_s0(i)*cos(alfa)-y_s0(i)*sin(alfa);
        y(i) = y_offset+x_s0(i)*sin(alfa)+y_s0(i)*cos(alfa);
        z(i) = z_offset+z_s0(i);
        
        [q1(i),q2(i),q3(i),ang_b(i),ang_tg(i)]=cinver(x(i),y(i),z(i),i);
    end;
end

%figure
figure (1)
subplot(3,1,1)
plot(q1*180/pi,'b','linewidth',2)
grid
title('q1')
subplot(3,1,2)
plot(q2*180/pi,'r','linewidth',2)
grid
title('q2')
subplot(3,1,3)
plot(q3*180/pi,'g','linewidth',2)
title('q3')
grid

figure (2)
subplot(3,1,1)
plot(x,'b','linewidth',2)
grid
title('x')
subplot(3,1,2)
plot(y,'r','linewidth',2)
grid
title('y')
subplot(3,1,3)
plot(z,'g','linewidth',2)
title('z')
grid

figure (5)
plot3(x,y,z)
grid
xlabel('x')
ylabel('y')
zlabel('z')
