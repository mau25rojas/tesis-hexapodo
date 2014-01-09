clc
clear all
Q = load ("SalidaQ.txt");

%figure
figure (3)
subplot(3,1,1)
plot((Q(:,3))*180/pi,'b','linewidth',2)
grid
title('q1')
subplot(3,1,2)
plot((Q(:,4))*180/pi,'r','linewidth',2)
grid
title('q2')
subplot(3,1,3)
plot((Q(:,5))*180/pi,'g','linewidth',2)
title('q3')
grid

X = load ("SalidaX.txt");

%figure
figure (4)
subplot(3,1,1)
plot((X(:,3)),'b','linewidth',2)
grid
title('x')
subplot(3,1,2)
plot((X(:,4)),'r','linewidth',2)
grid
title('y')
subplot(3,1,3)
plot((X(:,5)),'g','linewidth',2)
title('z')
grid

figure (6)
plot3(X(:,3),X(:,4),X(:,5))
grid
xlabel('x')
ylabel('y')
zlabel('z')