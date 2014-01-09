
function [x,y,z] = trayectoria(landa, dh, vel_apoyo, vel_transferencia, t, s)

switch s

    case 0
        x = -landa/2 + vel_apoyo*t;
        y = 0;
        z = -dh/2;
        
    case 1
        y = 0;
        x = landa/2;
        z = -dh/2 + vel_transferencia*t;
        
    case 2
        x = landa/2 - vel_transferencia*t;
        y = 0;
        z = dh/2;
        
    case 3
        x = -landa/2;
        y = 0;
        z = dh/2 - vel_transferencia*t;
    
    otherwise
        x=0;
        y=0;
        z=0;

end
