clc
clear 
close all

%% ------------------ Parametri MPC ------------------ %%

% ---- Prova n°1 buona: alte iterazioni, alto guadagno, alta finestra ----%

T_sim = 10;   
tau = 0.01; 
ngiri = 2;
max_iterations = round(T_sim / (tau * ngiri));
Nu = 10; 
weight_Q = 500;
weight_R = 1;

% ---- Prova n°2 interessante: basse iterazioni, alto guadagno, bassa finestra ----%

% T_sim = 10;               
% iterations = 100;
% Nu = 30; %50 interessante
% weight_Q = 500;
% weight_R = 10;
% ngiri = 4;
% tau = 0.01;
% max_iterations = ngiri*iterations;

% ---- Prova n°3 interessante: alte iterazioni, basso guadagno, alta finestra ---- %

% T_sim = 10;               
% iterations = 300;
% Nu = 70; 
% weight_Q = 10;
% weight_R = 10;
% ngiri = 4;
% tau = 0.01;
% max_iterations = ngiri*iterations;

time = linspace(0, T_sim, max_iterations);

n = 3;
max_theta = ngiri*2*pi;
r = 1;                            % m
omega = max_theta/T_sim;          % rad/s
v = omega*r;                      % m/s

%% ------------------ Traiettoria desiderata ------------------ %%
% i parametri relativi alle traiettorie sono definiti all'interno della
% funzione "genTrajectory.m"

% 1 --> Ellisse
% 2 --> Circonferenza
% 3 --> Parabola
% 4 --> Sinusoide
% 5 --> Onda quadra
% 6 --> Spirale
% 7 --> Lineare

tipo_traiettoria = 2;
center = [0; 0];
x_des = genTrajectory(n, max_iterations, tipo_traiettoria, ngiri, center, r);

%% ------------------ Definizione matrici MPC ------------------ %%
% questa dinamica dovrà essere sottratta alla x linearizzata del sistema, e
% data in pasto al controllore del modello mpc

x_k = zeros(n, max_iterations);
x_nl = zeros(n, max_iterations);

x0 = [1; 0; 0];
x0_nl = x0;
u0 = [0; 0];

x_k(:, 1) = x0;
x_nl(:, 1) = x0_nl;

[Ac, Bc, Pc] = LPV_MPC_System(x0, u0);

% in particolare, si può giocare con i pesi relativi alle matrici Q ed R
% per soddisfare i requisiti di controllo

Q = weight_Q*eye(n);
R = weight_R*eye(size(Bc, 2));
S = zeros(n, n);
U_star = zeros(max_iterations, size(Bc, 2));
A = Ac;
B = Bc;
C = ones(1, n);

u_max = [100; 100];
u_min = -u_max;

%% ------------------ LPV MPC ------------------ %%

for k = 1:max_iterations-1
    
    txt = sprintf('Iterazione n°: %d / %d', k, max_iterations);
    disp(txt);
    x_error = x_k(:, k) - x_des(:, k);
    x0 = x_error;
    
    u_mpc = LPV_MPC_Controller(x0, Nu, A, B, C, Q, R, S, u_max, u_min);
    [Ac, Bc, Pc] = LPV_MPC_System(x_k(:, k), u_mpc);
 
    % dinamica del sistema
    A = (eye(size(Ac, 2)) + tau*Ac);
    B = tau*Bc;
    P = tau*Pc;
    x_k(:, k+1) = A*x_k(:, k) + B*u_mpc + P;
    
    % dinamica non lineare del sistema preso in esame
    tspan = [time(k), time(k+1)];
    [t, x] = ode45(@(t, x) NL_System(x, u_mpc), tspan, x0_nl);
    x0_nl = x(end, :).';
    x_nl(:, k+1) = x0_nl;

    % controllo ottimo su tutto l'orizzonte temporale
    U_star(k, :) = u_mpc';
end

%% ------------------ Grafici ------------------ %%

figure(1)
hold on
plot(time, x_k);
plot(time, x_des);
grid on;
title("MPC States")
legend("x_{mpc}", "y_{mpc}", "θ_{mpc}", "x_{des}", "y_{des}", "θ_{des}");

figure(2)
hold on
plot(time, x_des - x_k);
grid on;
title("MPC Errors")
legend("x_{error}", "y_{error}", "θ_{error}");

figure(3)
plot(U_star);
grid on
title("MPC Controls")
legend("v_{mpc}", "ω_{mpc}");

figure(4)
hold on
plot(x_k(1, :), x_k(2, :));
plot(x_des(1, :), x_des(2, :));
plot(x_nl(1, :), x_nl(2,:));
grid on
title("Trajectory")
legend("MPC trajectory", "Desired trajectory", "Non-Linear trajectory");
