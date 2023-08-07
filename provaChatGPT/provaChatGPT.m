clc
clear all
close all

% Parametri del robot mobile
v_eq = 1.0; % Velocità di equilibrio (m/s)
L = 0.5; % Distanza tra il centro di massa del robot e l'asse di rotazione delle ruote (metri)

% Punto di equilibrio (stato e ingresso)
x_eq = 0.0;
y_eq = 0.0;
theta_eq = 0.0;
u_eq = [v_eq; 0]; % In questo caso, manteniamo la velocità angolare a zero.

% Orizzonte di predizione e controllo del MPC
N = 10; % Numero di passi di predizione
Q = diag([1, 1, 1]); % Peso per le variabili di stato (delta_x, delta_y, delta_theta)
R = diag([1, 0.1]); % Peso per le variabili di controllo (v, omega)

% Costruzione del modello LPV del sistema
A = [0, 0, -v_eq * cos(theta_eq);
     0, 0, -v_eq * sin(theta_eq);
     0, 0, 0];
B = [0, 0;
     0, 0;
     0, 1];

% Inizializzazione dello stato del robot
x0 = [0.1; 0.2; 0.05]; % Deviazione iniziale dalle variabili di stato del punto di equilibrio

% Loop di simulazione
T_sim = 5; % Tempo totale di simulazione (secondi)
dt = 0.1; % Intervallo di campionamento (secondi)
num_steps = T_sim / dt;

x_history = zeros(3, num_steps);
x = x0;

for k = 1:num_steps
    % Calcolo dell'ingresso di controllo utilizzando LPV-MPC
    u = LPV_MPC_Controller(x, u_eq, A, B, N, Q, R);

    % Simulazione del sistema utilizzando l'ingresso di controllo
    x = simulateRobotMobile(x, u, dt, v_eq, theta_eq, L);

    % Salvataggio dello stato corrente nella storia dello stato
    x_history(:, k) = x;
end

% Visualizzazione dei risultati
t = 0:dt:T_sim-dt;
plot(t, x_history(1, :), 'b', t, x_history(2, :), 'r', t, x_history(3, :), 'g');
legend('delta_x', 'delta_y', 'delta_theta');
xlabel('Tempo (s)');
ylabel('Deviazione dalle variabili di stato');
title('Simulazione LPV-MPC per il robot mobile nel piano');
grid on;
