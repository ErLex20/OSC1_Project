
clc
clear 
close all

% simulazione del problema della brachistocrona

theta = -pi:0.001:0;
z = zeros(1, length(theta));
t = zeros(1, length(theta));
n = length(theta);
ti = 0;
tf = 100;
R = (ti - tf)/pi;
P = tf;

z1 = z;
z2 = z;
z_dot = z;
z2_dot = z;

% brachistochrone construction
for i = 1:n
    z(i) = R*(1+cos(theta(i)));
    t(i) = -R*(theta(i) + sin(theta(i))) + P;
end

% parametri delle rimanenti curve
ca = z(n)/t(n);
a = (z(n) - z(1))/sqrt(tf-ti);
q = z(1);
h = ti;

% calcolo delle curve retta e radice quadrata.
% calcolo dei coefficienti angolari tangenti alla brachistocrona e radice
% quadrata
for i = 1:n
    z1(i) = ca*t(i);
    z2(i) = a*sqrt(t(i)- h) + q;
    z_dot(i) = -sqrt((2*R - z(i))/z(i));
    z2_dot(i) = a/(2*sqrt(t(i) - h));
end

% inserita in quanto sarebbe inf
z_dot(1) = -300;
z2_dot(1) = -300;

% for i = 1:n
%     y(i) = z_dot(i0)*t(i) + z2(i0) - z_dot(i0)*t(i0);
% end

% simulazione tempo di percorrenza di una massa puntiforme sulle
% traiettorie prese in esame, soggetta unicamente a forza di gravità

x = zeros(1, n); 
y =x;
x1 = zeros(1, n);
x2 = zeros(1, n);
delta_t = 0.01;         % trovato sperimentalmente 
k = 3;
x(1) = ti;              % m
x(2) = ti;
x1(1) = ti;
x1(2) = ti;
x2(1) = ti;
x2(2) = ti;
m = 1;                  % kg
g = 9.81;               % m/s^2
%rivedere la posizione con approssiamzione della derivata


while(abs(x(k-2)) < tf )
   [v, id] = min(abs(x(k-2)-t));
   x(k) = 2*x(k-1) - x(k-2) + (delta_t^2)*m*g*abs(sin(z_dot(id)));
   y(k) = 2*y(k-1) - y(k-2) + (delta_t^2)*m*g*abs(cos(z_dot(id)));
   % x(k) = x(k-1) + (delta_t^2)*m*g*abs(sin(z_dot(k-1)));
   k = k+1; 
end
T = delta_t*k;
k = 3;
while(abs(x1(k-2)) < tf )
    x1(k) =  2*x1(k-1) - x1(k-2) + (delta_t^2)*m*g*abs(sin(ca));
    k = k+1;
end
T1 = delta_t*k;
k = 3;
while(abs(x2(k-2)) < tf )
    [v, id] = min(abs(x(k-2)-t));
    x2(k) = 2*x2(k-1) - x2(k-2) + (delta_t^2)*m*g*abs(sin(z2_dot(id)));
    k = k+1;
end
T2 = delta_t*k;


%legend('brachistochrone', 'rect', 'square root');

disp('----------------------------------------------FINAL RESULTS----------------------------------------------');
disp('Brachistochrone curve travel time:');
disp(T);

disp('Straight line curve travel time:');
disp(T1);

disp('Square root curve travel time:');
disp(T2);


% simulazione video?
figure(1)
hold on
plot(t, z,'LineWidth',2);
plot(t, z1);
plot(t, z2);
grid on

for i = 0:n
    patch(2, 2, 'r');
    drawnow

end









