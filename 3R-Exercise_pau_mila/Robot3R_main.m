% Exercici Robot 3R

% Parametres donats per l'anunciat

Pini = [0.9; -0.2]; % Punt inicial del end effector (Start) 
Pfin = [0.9; -0.7]; % Punt final del end effector (Goal) 
O = 0; % Comencem amb angle 0º, les funcions estan creades segons la primera posicio

v=-0.1; % Velocitat del Robot, aquesta variable es pot cambiar
T = [0; v; 0]; % Twist que nomes té velocitat en el eix -y
temps = 0.01; % Temps

% Links del robot

L1 = 0.62; 
L2 = 0.57;

g1 = 0.1;
g2 = 0.2;
g3 = 0.3;

n = round((Pfin(2)-Pini(2))/(v*temps)); % Iteracions que haurem de calcular les posicions i angles de les joints, per arribar al goal

% Variables que utilitzem en el programa

k=1; % La variable que utilitzarem per fer totes les iteracions i per saber quines dades hem d'utilitzar

% La posicio de les joints

joint1 = []; 
joint2 = [];
joint3 = [];

% El angle de les joints

a1 = [];
a2 = [];
a3 = [];

%Velocitat de rotaci de les diferents joints

G = []; 

% Posicio del end effector

P = [];

% Busquem els angles de les diferents joints amb la funcio inversepos

[a1(k),a2(k),a3(k)] = inversepos (Pini,O);

% Posicio de les joints inicial

joint1(:,k) = [0; 0];             
joint2(:,k) = joint1(:,k) + L1*[cos(a1(k));sin(a1(k))];
joint3(:,k) = joint2(:,k) + L2*[cos(a1(k)+a2(k));sin(a1(k)+a2(k))];

for k = 1:n

% Busquem la jacobianade les joints amb la funcio jacobiana    
    
jac = jacobian(joint1(:,k),joint2(:,k),joint3(:,k));

% Calculem la velocitat de les joints amb el jacobia i twist amb la
% inversekin

G(:,k+1) = inversekin(T,jac);

% Calculem els angles de les diferents joints

a1(k+1) = a1(k) + temps*G(1,k);
a2(k+1) = a2(k) + temps*G(2,k);
a3(k+1) = a3(k) + temps*G(3,k);

% Calculem les posicions de les joints
    
joint1(:,k+1) = joint1(:,k);
joint2(:,k+1) = joint1(:,k) + L1*[cos(a1(k)); sin(a1(k))];
joint3(:,k+1) = joint2(:,k) + L2*[cos(a1(k)+a2(k)); sin(a1(k)+a2(k))];
    
% Calculem la posició del end effector

P(:,k+1) = joint3(:,k) + [g1+g3;-g2];

% Plotagem el robot 3R, les diferents joints i links

figure(1)
axis([-0.2 1 -1 0.2])

% Links

plot([joint1(1,k),joint2(1,k)],[joint1(2,k),joint2(2,k)],'b')

hold on
title('Robot 3R')
xlabel('x(m)')
ylabel('y(m)')

plot([joint2(1,k),joint3(1,k)],[joint2(2,k),joint3(2,k)],'-b')
plot([joint3(1,k),joint3(1,k)+g1],[joint3(2,k),joint3(2,k)],'-b')
plot([joint3(1,k)+g1,joint3(1,k)+g1],[joint3(2,k),joint3(2,k)-g2],'-b')
plot([joint3(1,k)+g1,joint3(1,k)+g1+g3],[joint3(2,k)-g2,joint3(2,k)-g2],'-b')

% Joints

plot(joint1(1,k),joint1(2,k),'ok','MarkerFaceColor','k')
plot(joint2(1,k),joint2(2,k),'ok','MarkerFaceColor','k')
plot(joint3(1,k),joint3(2,k),'ok','MarkerFaceColor','k')
    
axis([-0.2 1 -1 0.2])
hold off

end

% Plotagem la velocitat de les joints en el temps

% Calculem tot el temps durant les diverses iteracions n

t = linspace(temps,(k+1)*temps,k+1);

figure(2)
hold on

title(['Joint speeds'])
xlabel(['t(s)'])
ylabel(['speed(rad/s)'])
axis([0 (k+1)*temps -1.5 1.5])

plot(t,G(1,:))
plot(t,G(2,:))
plot(t,G(3,:))

hold off
legend('joint1_velocity','joint2_velocity','joint3_velocity')

