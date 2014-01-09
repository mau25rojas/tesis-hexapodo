% Cinematica inversa

function [q1, q2, q3,a_beta,a_tg] = cinver(x,y,z,i)
	
L1 = 0.052;
L2 = 0.06;
L3 = 0.105;
a_beta=0; a_tg=0; 

    px = x;
    py = y;
    pz = z;

    % calculo q1---------------
          %q1 = atan2(py,px);
          q1 = atan2(py,px)-pi/2;
   % ---------------------------------
   L23_p = sqrt(px^2 + py^2) - L1;
   L23 =sqrt(L23_p*L23_p + pz^2);

    % calculo q3 ------------------------
    arg = (L2^2 + L3^2 - L23^2)/(2*L2*L3);
    if (abs(arg)>1)
        fprintf('ERROR acos q3 %.1f L23:%.2f\n',i,L23)
        q1=0;q2=0;q3=0;
        return;
    end
    a_beta = acos(arg);
    %q3 = pi - a_beta;
    %q3 = a_beta;
    q3 = a_beta-pi/2;
    
   % ---------------------------------
   
   % calculo q2 -----------------------------
   arg1= (L3/L23)*sin(a_beta);
    if (abs(arg1)>1)
        fprintf('ERROR asin q2 %.1f\n',i)
        q1=0;q2=0;q3=0;
        return;
    end
    teta = atan(-pz/L23_p);
    gamma = asin(arg1);
    a_tg = teta - gamma;
	%q2 = pi/2 - (a_tg);
    q2 = teta - gamma;
   % ---------------------------------
end