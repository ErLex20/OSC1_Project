clc
clear all 
close all

% definisco la dinamica dell'uniciclo;

v = 1;            % m/s
omega = 1;          % rad/s

% ------------- variabili per i requisiti ------------- %

% ---- Prova n°1 buona: alte iterazioni, alto guadagno, alta finestra --------%

T_sim = 10;               
iterations = 300;
Nu = 60; 
weight_Q = 500;
weight_R = 10;
ngiri = 6;

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


% definisco la dinamica non lineare dell'uniciclo


x = zeros(max_iterations, 3);


%% ---- Traiettoria desiderata: Circonferenza ---------- %

theta = linspace(0, ngiri*2*pi, max_iterations);


x_des = zeros(3, max_iterations);
r = 1;                          % Raggio della circonferenza
center = [0;0];
for i = 1:max_iterations
    x_des(1, i) = center(1, 1)+r*cos(theta(i));
    x_des(2, i) = center(2, 1)+r*sin(theta(i));
    x_des(3, i) = theta(i);
end

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

u_max = [1;1.2];
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
    tspan = [theta(k), theta(k) + tau];
    odefun = @(t_nl, dxdt) [uk_f(1) * cos(dxdt(3));uk_f(1) * sin(dxdt(3)); uk_f(2)];
    %odefun = @(t_nl, dxdt) [v * cos(dxdt(3));v * sin(dxdt(3)); omega];
    [t_nl, x_temp] = ode45(odefun, tspan, x_start);
    x_start = x_temp(end, :).';
    x(k, :) = x_start.';
    
    % controllo ottimo su tutto l'orizzonte temporale
    U_star(k,:) = uk_f';
end
x(end, :) = [1,0,0];

% Plot dei risultati (ad esempio, la traiettoria)


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
plot(x(:, 1), x(:, 2));
title('Trajectory of a Uniform Circular Movement')
grid on
legend("MPC trajectory","desired trajectory", "non-linear trajectory");




