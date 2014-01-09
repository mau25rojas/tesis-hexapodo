clear all
clc

%Trabajemos en cm
L1 = 5;
L2 = 6;
L3 = 10.5;

inicio=-22;
fin=22;
paso=1;
max_ang_q1 = 65*pi/180;
min_ang_q1 = -85*pi/180;
max_ang_q2 = 40*pi/180;
min_ang_q2 = -90*pi/180;
max_ang_q3 = 90*pi/180;
min_ang_q3 = -35*pi/180;

posicion_out=[];
posicion_mal=[];
% --------
pz=0;
   
for px=inicio:paso:fin
    for py=inicio:paso:fin
       for pz=inicio:paso:fin
   % ---------------------------------
   L23_p = sqrt(px^2 + py^2) - L1;
   L23 = sqrt(L23_p^2 + pz^2);

   % calculo q1 ------------------------
    q1 = atan2(px,py); %da radianes
%     q1_deg=q1*180/pi;
    if (q1 < min_ang_q1 || max_ang_q1 < q1)   %fuera de rango
        vector_xyz = [px; py; pz];
        posicion_mal=[posicion_mal vector_xyz];
    else
       % calculo q3 ------------------------
        Num = L2^2 + L3^2 - L23^2;
        Den = 2*L2*L3;
        arg = Num/Den;
        if (abs(arg)>1)
            vector_xyz = [px; py; pz];
            posicion_mal=[posicion_mal vector_xyz];
        else
            a_beta = acos(arg);
            q3 = pi - a_beta;
%             q2_deg=q3*180/pi;
            if (q3 < min_ang_q3 || max_ang_q3 < q3)   %fuera de rango
                vector_xyz = [px; py; pz];
                posicion_mal=[posicion_mal vector_xyz];
            else
                % calculo q2 -----------------------
                arg1= (L3/L23)*sin(a_beta);
                if (abs(arg1)>1)
                    vector_xyz = [px; py; pz];
                    posicion_mal=[posicion_mal vector_xyz];
                else
                    teta = atan(-pz/L23_p);
                    gamma = asin(arg1);
                    a_tg = teta - gamma;
                    q2 = pi/2 - (a_tg);
%                     q2_deg=q2*180/pi;
                    if (q2 < min_ang_q2 || max_ang_q2 <q2)   %fuera de rango
                        vector_xyz = [px; py; pz];
                        posicion_out=[posicion_out vector_xyz];
                    end
                end
            end
        end
    end
        end     %end de fors
    end
end
       % ---------------------------------