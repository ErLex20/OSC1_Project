clc
clear 
close all

%% Sistema LPV MPC 
% Generazione traiettoria
R = 5; % Raggio della traiettoria circolare [m]
[x_des, y_des, theta_des] = GenerateCircularTrajectory(R);

% Punto di equilibrio
xc = [100; 50; 30];
uc = [0; 0];
[A, B, P] = LPV_MPC_System(xc, uc);
C = eye(size(A,1));

% Orizzonte di predizione e controllo del MPC
N = 6; % Numero di passi di predizione
Q = diag([1, 1, 1]); % Peso per le variabili di stato (delta_x, delta_y, delta_theta)
R = diag([1, 0.1]); % Peso per le variabili di controllo (v, omega)

% Loop di simulazione
T_sim = 10; % Tempo totale di simulazione [s]
dt = 0.1; % Intervallo di campionamento [s]
num_steps = T_sim / dt;

delta_xk_history = zeros(size(A,1), num_steps);

%% Simulazione

for k = 1:num_steps
    % Costruzione del modello LPV del sistema
    xk_des = [x_des(k); 
              y_des(k);
              theta_des(k)];
    delta_xk = xk_des - xc;

    [A, B, P] = LPV_MPC_System(delta_xk, uc);

    % Calcolo dell'ingresso di controllo utilizzando LPV-MPC
    u = LPV_MPC_Controller(delta_xk, N, A, B, C, Q, R);

    % Salvataggio dello stato corrente nella storia dello stato
    delta_xk_history(:, k) = LPV_MPC_Simulation(delta_xk, u, dt*k, A, B, P);

    % Aggiornamento punto di equilibrio
    uc = uc + u;
    xc = A*xc + B*uc;
end

%% Visualizzazione dei risultati
t = 0:dt:T_sim-dt;
figure(1)
plot(t, delta_xk_history(1, :), 'b', t, delta_xk_history(2, :), 'r', t, delta_xk_history(3, :), 'g');
title("Simulazione LPV-MPC per il robot mobile nel piano");
legend("Δx", "Δy", "Δθ");
xlabel("Tempo [s]");
ylabel("Deviazione dalle variabili di stato");
grid on;
