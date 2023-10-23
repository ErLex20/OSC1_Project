clc
clear 
close all


%% ------------------ Parametri MPC ------------------ %%

T_sim = 10;   
tau = 0.01; 
ngiri = 4;
max_iterations = round(T_sim* ngiri / (5*tau ));
Nu = 5; 
weight_Q = 600;   %3e2        
weight_R = 1;

time = linspace(0, T_sim, max_iterations);

n = 3;
p = 2;
max_theta = ngiri*2*pi;
r = 0.1;                          % m
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
x_des = genTrajectory(n, max_iterations, tipo_traiettoria, ngiri, center, 5*r);

%% ------------------ Definizione matrici MPC ------------------ %%
% questa dinamica dovrà essere sottratta alla x linearizzata del sistema, e
% data in pasto al controllore del modello mpc

%x_nl = zeros(n, max_iterations);

%x0 = [1; 0; pi/2];
x0_nl = [2; 0; pi/2];
u0 = [1; 0];

x_nl(:, 1) = x0_nl;

% in particolare, si può giocare con i pesi relativi alle matrici Q ed R
% per soddisfare i requisiti di controllo

Q = weight_Q*eye(n);
%Q = [1000,0,0;0,1000,0;0,0,1];
R = weight_R*eye(p);
S = zeros(n, n);
U_star = zeros(max_iterations, p);

u_mpc = u0;

u_max = [100; 100];
u_min = -u_max;
x_des_o = x_des;
%x_des = [x_des, [x_des_o(1, end)*ones(1,Nu);x_des_o(2, end)*ones(1,Nu);x_des_o(3, end)*ones(1,Nu)]];
x_des = genTrajectory(n, max_iterations + Nu, tipo_traiettoria, ngiri, center, 5*r);

%% ------------------ LPV MPC ------------------ %%
% Versione Lineare
% for k = 1:max_iterations-1
%     txt = sprintf('Iterazione n°: %d / %d', k, max_iterations);
%     disp(txt);
%     x_error = x_k(:, k) - x_des(:, k);
%     x0 = x_error;
%     
%     u_mpc = LPV_MPC_Controller(x0, Nu, A, B, C, Q, R, S, u_max, u_min);
%     [Ac, Bc, Pc] = LPV_MPC_System(x_k(:, k), u_mpc);
% 
%     % dinamica del sistema
%     A = (eye(size(Ac, 2)) + tau*Ac);
%     B = tau*Bc;
%     P = tau*Pc;
%     x_k(:, k+1) = A*x_k(:, k) + B*u_mpc + P;
%     
%     % dinamica non lineare del sistema preso in esame
%     tspan = [time(k), time(k+1)];
%     [t, x] = ode45(@(t, x) NL_System(x, u_mpc), tspan, x0_nl);
%     x0_nl = x(end, :).';
%     x_nl(:, k+1) = x0_nl;
% 
%     % controllo ottimo su tutto l'orizzonte temporale
%     U_star(k, :) = u_mpc';
% end

%% ------------------ LPV MPC ------------------ %%
% Versione Non-Lineare
for k = 1:max_iterations-1
    txt = sprintf('Iterazione n°: %d / %d', k, max_iterations);
    disp(txt);
    u_mpc = LPV_MPC_Controller(x_nl(:,k), u_mpc, x_des, Nu, Q, R, S, u_max, u_min, k, tau);
    % dinamica del sistema non lineare
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
plot(time, x_nl);
plot(time, x_des_o);
grid on;
title("MPC States")
legend("x_{nl}", "y_{nl}", "θ_{nl}", "x_{des}", "y_{des}", "θ_{des}");

figure(2)
hold on
plot(time, x_des_o - x_nl);
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
plot(x_des_o(1, :), x_des_o(2, :));
plot(x_nl(1, :), x_nl(2,:));
grid on
title("Trajectory")
legend("Desired trajectory", "Non-Linear trajectory");

%% Definisci i margini del grafico

figure(5)
hold on
axis equal
grid on

ax = gca;

h = hgtransform('Parent', ax);
j = hgtransform('Parent', ax);

% Disegna robot di traiettoria desiderata
pl1 = rectangle('Position', [x_des_o(1, 1)-r, x_des_o(2, 1)-r, 2*r, 2*r],...
                      'Curvature', [1 1],...
                      'EdgeColor', 'k',...
                      'LineWidth', 1,...
                      'LineStyle', '-',...
                      'FaceColor', [0.9290 0.6940 0.1250 0.5],...
                      'Parent', h);

% Disegna robot di traiettoria reale non lineare
pl2 = rectangle('Position', [x_nl(1, 1)-r, x_nl(2, 1)-r, 2*r, 2*r],...
                      'Curvature', [1 1],...
                      'EdgeColor', 'k',...
                      'LineWidth', 1,...
                      'LineStyle', '-',...
                      'FaceColor', [0 0.5 1 0.5],...
                      'Parent', j);

% Crea gli oggetti di linea per l'effetto scia
trail1 = line('XData', [], 'YData', [], 'Color', [0.9290 0.6940 0.1250, 0.5], 'LineWidth', 2, 'Parent', h);
trail2 = line('XData', [], 'YData', [], 'Color', [0 0.5 1, 0.5], 'LineWidth', 2, 'Parent', j);

hold off

% Inizia l'animazione
for k = 2:length(x_des_o) %xdes

    set(pl1, 'Position', [x_des_o(1, k)-r, x_des_o(2, k)-r, 2*r, 2*r]);
    set(pl2, 'Position', [x_nl(1, k)-r, x_nl(2, k)-r, 2*r, 2*r]);

    x_trail1 = [get(trail1, 'XData'), x_des_o(1, k)];
    y_trail1 = [get(trail1, 'YData'), x_des_o(2, k)];
    x_trail2 = [get(trail2, 'XData'), x_nl(1, k)];
    y_trail2 = [get(trail2, 'YData'), x_nl(2, k)];
    
    % Aggiorna la posizione delle linee di scia
    set(trail1, 'XData', x_trail1, 'YData', y_trail1);
    set(trail2, 'XData', x_trail2, 'YData', y_trail2);

    xlim([-2, 2]);
    ylim([-2, 2]);

    drawnow
end
