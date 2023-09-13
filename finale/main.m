clc
clear all 
close all


% ------------- variabili per i requisiti ------------- %

% ---- Prova n°1 buona: alte iterazioni, alto guadagno, alta finestra --------%

T_sim = 10;               
iterations = 300;
Nu = 60; 
weight_Q = 500;
weight_R = 10;
ngiri = 2;

% ---- Prova n°2 interessante basse iterazioni, alto guadagno, bassa finestra --------%

% T_sim = 10;               
% iterations = 100;
% Nu = 30; %50 interessante
% weight_Q = 500;
% weight_R = 10;
% ngiri = 4;

% ---- Prova n°3 interessante alte iterazioni, basso guadagno, alta finestra --------%

% T_sim = 10;               
% iterations = 300;
% Nu = 70; 
% weight_Q = 10;
% weight_R = 10;
% ngiri = 4;



max_iterations = ngiri*iterations;



% ----------------------------------------------------- %
t = linspace(0, T_sim, max_iterations);
tau = 0.01; % trovato sperimentalmente
time = linspace(0, T_sim, max_iterations);
% definisco la dinamica dell'uniciclo;

% v = 1;            % m/s
max_theta = ngiri*2*pi;
r = 1;
omega = max_theta/T_sim;          % rad/s
v = omega*r;            % m/s

% definisco la dinamica non lineare dell'uniciclo


x = zeros(max_iterations, 3);

%% ---- Traiettoria desiderata: ---------- %
% i parametri relativi alle traiettorie sono definiti all'interno della
% funzione "genTrajectory.m"

% 1 --> Ellisse
% 2 --> Circonferenza
% 3 --> Parabola
% 4 --> Sinusoide
% 5 --> Onda quadra
% 6 --> Spirale

tipo_traiettoria = 2;
%r = 1;                                       % Raggio della circonferenza
center = [0;0];
x_des = genTrajectory(max_iterations,tipo_traiettoria,ngiri,center,r);


% ---- Calcolo del controllo ------------------ %
%keyboard;
% questa dinamica dovrà essere sottratta alla x linearizzata del sistema, e
% data in pasto al controllore del modello mpc

xk_f = zeros(3, max_iterations);
xe_f = zeros(3, max_iterations);
x0 = [1;0;0];
xk_f(:, 1) = x0;
[Ac, Bc, Pc] = LPV_MPC_System(x0,[0;0]);    %(stato, controllo)

% -----definizione delle matrici relative all'indice di costo ----%
% in particolare, si può giocare con i pesi relativi alle matrici Q ed R
% per soddisfare i requisiti di controllo

Q = weight_Q*eye(3);
R = weight_R*eye(size(Bc, 2));
S = zeros(3, 3);
U_star = zeros(max_iterations, size(Bc, 2));      %vettore dei controlli
A = Ac;
B = Bc;
C = eye(3);
x_start = x0;
y0 = [1;0;0];

u_max = [100;100];
u_min = -u_max;


for k = 1:max_iterations - 1
    
    txt = sprintf('Iterazione n°: %d', k);
    disp(txt);
    x_error = xk_f(:, k) - x_des(:,k);
    %x_error = xe_f(:,k);
    x0 = x_error;
    
    uk_f = LPV_MPC_Controller(x0, Nu, A, B, C, Q, R, S, u_max, u_min);
    [Ac, Bc, Pc] = LPV_MPC_System(xk_f(:, k),uk_f);
 
    %dinamica del sistema
    A = (eye(size(Ac, 2)) + tau*Ac);
    B = tau*Bc;
    ck = tau*Pc;
    xk_f(:, k+1) = A*xk_f(:, k) + B*uk_f + ck;
    
    %dinamica non lineare del sistema preso in esame
    tspan = [time(k), time(k+1)];
    [t_f, y] = ode45(@(t_f, y) unicicloODE(t_f, y, v, omega, tipo_traiettoria, T_sim), tspan, y0);
%     [t_f, y] = ode45(@(t_f, y) unicicloODE(t_f, y, uk_f(1), uk_f(2), tipo_traiettoria), tspan, y0);
    y0 = y(end, :).';
    y_f(:, k) = y0;
    % controllo ottimo su tutto l'orizzonte temporale
    U_star(k,:) = uk_f';
end
y(end+1, :) = y0;



figure(1)
hold on
plot(t, xk_f);
plot(t, x_des);
grid on;
legend("x_{mpc}", "y_{mpc}", "theta_{mpc}", "x_{des}", "y_{des}", "theta_{des}");


figure(2)
plot(t, U_star);
title('Control system')
grid on
legend("v_{mpc}", "w_{mpc}");

figure(3)
hold on
plot(xk_f(1, :), xk_f(2, :));
plot(x_des(1, :),x_des(2, :));
plot(y_f(1, :), y_f(2,:));
title('Trajectory of a Uniform Circular Movement')
grid on
legend("MPC trajectory","desired trajectory", "non-linear trajectory");

function dydt = unicicloODE(t, y, v, omega, traiettoria, T_sim)
    x = y(1);
    y_t = y(2);    
    theta = y(3)+pi/2;
    
    dxdt = v * cos(theta);
    dydt = v * sin(theta);
    dthetadt = omega;
    dydt = [dxdt; dydt; dthetadt];
end


