L1 = 0.05;
L2 = 0.06;
L3 = 0.105;

Pos_inicio=-0.1;
Pos_fin=0.1;
ps = 0.001;
t = Pos_inicio:ps:Pos_fin-ps;
ceros = zeros(1,length(t));
Px = t;
Py = ceros;
Pz = ceros;

% Definiciones
L23_2 = (sqrt(Px.^2 + Py.^2)-L1).^2 + Pz.^2;
L23 = sqrt(L23_2);

%% Parametrización de curvas para q2
% sen(beta)
    % Limite para beta: 
    % ..-1 <= beta <= 1
    % ..-1 <= (L2^2+L3^2-L23^2)/(2*L2*L3) <= 1
    % ..-2*L2*L3-L2^2-L3^2 <= -L23^2 <= 2*L2*L3-L2^2-L3^2
    % .. 2*L2*L3+L2^2+L3^2 >= L23^2 >= -2*L2*L3+L2^2+L3^2
    Limite_beta_inf = (-2*L2*L3+L2^2+L3^2).*ones(1,length(t));
    Limite_beta_sup = (2*L2*L3+L2^2+L3^2).*ones(1,length(t));
    plot(t,Limite_beta_sup,'g');
    hold
    plot(t,L23_2,'r');
    plot(t,Limite_beta_inf,'b');
    grid
    xlabel('tiempo')
    ylabel('Limite beta')
    legend('Limite beta superior','L23^2','Limite beta inferior')
    
% asin((L3/L23)*sin(beta))
    % Limite para asin
    % ..suponiendo que "sin(beta)" ok!
    % ..-1 <= (L3/L23)*sin(beta) <= 1
    % ..-L3*sin(beta) >= L23 >= L3*sin(beta)
    beta = acos((L2^2 + L3^3 - L23.^2)./(2*L2*L3));
    figure(2)
    plot(t,beta)
    grid
    xlabel('tiempo')
    ylabel('beta')
   
    Limite_asin_sup = -L3.*sin(beta);
    Limite_asin_inf = L3.*sin(beta);
    figure(3)
    plot(t,Limite_asin_sup,'g');
    hold
    plot(t,L23,'r');
    plot(t,Limite_asin_inf,'b');
    grid
    xlabel('tiempo')
    ylabel('Limite asin')
    legend('Limite asin sup','L23','Limite asin inf')

