clc
clear 
close all

%% Sistema LPV MPC 
% Parametri robot
robot_length = 0.5;

% Generazione traiettoria
trajectory = "Circular";

%[x_des, y_des, theta_des] = GenerateTrajectory(trajectory, 100);

% Plot della traiettoria con l'orientamento del robot
% figure(4);
% plot(x_des, y_des, 'b-');
% hold on;
% quiver(x_des, y_des, cos(theta_des), sin(theta_des), 0.5, 'r');  % Rappresenta l'orientamento con frecce rosse
% axis equal;
% xlabel('X');
% ylabel('Y');
% title('Traiettoria del Robot Mobile con Orientamento Ortogonale alla Traiettoria');
% grid on;
% hold off;


% condizioni iniziali
x0 = [10; 5; 0];
% controllo iniziale
u_d = [0; 1]; 

% Intervallo di tempo per la simulazione
tspan = [0, 9]; 

% Risoluzione delle equazioni differenziali per ottenere i valori degli stati dinamici
[t, x] = ode45(@(t, x) unicycle_dynamics(t, x, u_d), tspan, x0);
% figure(1)
% plot(t, x)
% grid on
% legend("x", "y", "theta");


% quella sopra rappresenta la dinamica non lineare dell'uniciclo.
% questa dovrà essere linearizzata. (delta_x = x_nl - xc) , (delta_u = u - uc), dove (xc, uc) è il
% punto di "equilibrio" scelto.
x_mpc = zeros(3, size(t, 1));
u_mpc = zeros(size(t,1),2);
tau = 0.1;
for tc = 1:size(t,1)
    xc = x(:,tc);
    [Ac, Bc, Pc] =  LPV_MPC_System(xc, u_mpc(:,tc));
    C = eye(size(Ac,2));
    Q = diag([1, 1, 1]); 
    R = diag([1, 0.1]); 
    u = LPV_MPC_Controller(x0, size(t,1), Ac, Bc, C, Q, R);
    u_mpc(:,tc) = [u(1);u(2)];
    x_mpc(:,tc + 1) = (eye(size(Ac,2)) + tau*Ac)*x_mpc(:, tc) + tau*Bc*u_mpc(:,tc);
end









%%


% Punto di equilibrio
xc = [10; 5; 3];
uc = [0; 0];

[A, B, P] = LPV_MPC_System(xc, uc);
C = eye(size(A,1));

% Orizzonte di predizione e controllo del MPC
N = 2; % Numero di passi di predizione
Q = diag([1, 1, 1]); % Peso per le variabili di stato (delta_x, delta_y, delta_theta)
R = diag([1, 0.1]); % Peso per le variabili di controllo (v, omega)

% Loop di simulazione
T_sim = 10; % Tempo totale di simulazione [s]
dt = 0.1; % Intervallo di campionamento [s]
num_steps = T_sim / dt;

delta_xk_history = zeros(size(A,1), num_steps);
u_history = zeros(size(B,2), num_steps);

%% Simulazione

ngiri = 0;
for k = 1:num_steps
    if(k > 1)
        [theta_des(k), ngiri] = CorrectionTrajectory(x_des(k), y_des(k),theta_des(k-1), ngiri);
    end

    % Costruzione del modello LPV del sistema
    xk_des = [x_des(k); 
              y_des(k);
              theta_des(k)];
    delta_xk = xk_des - xc;

    [Ac, Bc, P] = LPV_MPC_System(delta_xk, uc);

    % Calcolo dell'ingresso di controllo utilizzando LPV-MPC
    u = LPV_MPC_Controller(delta_xk, N, Ac, Bc, C, Q, R);

    % Salvataggio dello stato corrente nella storia dello stato
    delta_xk_history(:, k) = LPV_MPC_Simulation(delta_xk, u, dt, A, B, P);
    u_history(:, k) = u;

    % Aggiornamento punto di equilibrio
    uc = uc + u;
    xc = A*xc + B*uc;
end

cost = LPV_MPC_CostIndex(Q, R, delta_xk_history, u_history);

% %% Visualizzazione dei risultati
% % grafico della deviazione dalle variabili di stato
% t = 0:dt:T_sim-dt;
% figure(1)
% plot(t, delta_xk_history(1, :), 'b', t, delta_xk_history(2, :), 'r', t, delta_xk_history(3, :), 'g');
% title("Simulazione LPV-MPC per il robot mobile nel piano");
% legend("Δx", "Δy", "Δθ");
% xlabel("Tempo [s]");
% ylabel("Deviazione dalle variabili di stato");
% grid on;

% % grafico delle traiettorie
% figure(2);
% hold on;
% 
% % Plot della traiettoria desiderata
% plot(x_des, y_des, 'b-');
% for i = 1:num_steps
%     x = delta_xk_history(1, i);
%     y = delta_xk_history(2, i);
%     plot(x, y, '*');
%     %theta_des(i) = atan2(y_des(i+1) - y_des(i), x_des(i+1) - x_des(i));
%     robot_corners = [x - robot_length/2, x + robot_length/2, x + robot_length/2, x - robot_length/2;
%                      y - robot_length/2, y - robot_length/2, y + robot_length/2, y + robot_length/2];
%     R = [cos(theta_des(i)), -sin(theta_des(i)); sin(theta_des(i)), cos(theta_des(i))];
%     rotated_corners = R * robot_corners;
%     plot(rotated_corners(1,:), rotated_corners(2,:), 'r-');
% end
% title('Plot del Robot Mobile');
% legend("Traiettoria desiderata", "Traiettoria baricentro robot", "Traiettoria orientamento robot")
% xlabel('X');
ylabel('Y');
grid on;
hold off;

% grafico del costo
figure(3);
plot(cost, 'b-');
title('Evoluzione del costo');
legend("Costo");
xlabel('Tempo [t]');
ylabel('Costo');
grid on;