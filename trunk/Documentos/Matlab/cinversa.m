L1=0.052;
L2=0.06;
L3=0.105;

px= 0.045;
py= 0.1;
pz= -0.06;


   % //------calculo q1------
    %//p[0] = atan2(px,py);
    q1 = atan2(py,px)*180/pi - 90
    %//----------------------

    L23_aux = sqrt(px*px + py*py) - L1;
    L23 = sqrt(L23_aux*L23_aux + pz*pz);

    %//------calculo q3------
	arg = (L2*L2 + L3*L3 - L23*L23)/(2*L2*L3);
	if abs(arg)>=1 
		arg=0
	end
	beta1 = acos(arg);
	
   % //El ajuste de pi se hace para coincidir con eje de D-H
    q3 = (beta1)*180/pi
    %//p[2] = beta;
    %//----------------------

    %//------calculo q2------
	arg1= (L3/L23)*sin(beta1);
	if abs(arg1)>=1
        		arg1=0
	end
		teta = atan(-pz/L23_aux);
		gamma = asin(arg1);
	
	%/El ajuste de pi/2 se hace para coincidir con eje de D-H
%    //p[1] = pi/2 - (teta-gamma);
    q2 = (teta-gamma)*180/pi