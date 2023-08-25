clc
clear all 
close all

% definisco la dinamica dell'uniciclo;

v = 0.1;            % m/s
omega = 0.1;          % rad/s
T_sim = 10;         % s
max_iterations = 1000;
Nu = 60; % numero di passi di predizione;
%incr_t = T_sim/len;
theta = linspace(0, 2*pi, max_iterations);
t = linspace(0, T_sim, max_iterations);
tau = 0.01;


close all
% definisco ora la traiettoria desiderata dell'uniciclo, ad esempio una
% circonferenza;
x_des = zeros(3, max_iterations);
r = 1;                          % Raggio della circonferenza
x_start = [1;0;0];
x_des(:, 1) = x_start;
for i = 2:max_iterations
    x_des(1, i) = r*cos(theta(i));
    x_des(2, i) = r*sin(theta(i));
    x_des(3, i) = theta(i);
end


%keyboard;
% questa dinamica dovrà essere sottratta alla x linearizzata del sistema, e
% data in pasto al controllore del modello mpc

xk_f = zeros(3, max_iterations);
xe_f = zeros(3, max_iterations);
x0 = [1;0;0];
xk_f(:, 1) = x0;
[Ac, Bc, Pc] = LPV_MPC_System(x0,[0;0]);    %(stato, controllo)
Q = eye(3);
R = eye(size(Bc, 2));
U_star = zeros(max_iterations, size(Bc, 2));      %vettore dei controlli
A = Ac;
B = Bc;
C = eye(3);
for k = 1:max_iterations-1
    x_error = xk_f(:, k) - x_des(:,k);  
    %x_error = xe_f(:,k);
    x0 = x_error;
    uk_f = LPV_MPC_Controller(x0, Nu, A, B, C, Q, R);
    [Ac, Bc, Pc] = LPV_MPC_System(xk_f(:, k),uk_f);
 
    %dinamica del sistema
    
    A = (eye(size(Ac, 2)) + tau*Ac);
    B = tau*Bc;
    ck = tau*Pc;
    xk_f(:, k+1) = A*xk_f(:, k) + B*uk_f + ck;

    % dinamica di errore
    Ae = A;
    Be = -B;
    cke = -tau*A*x_des(:, k) - ck;
    xe_f(:, k+1) = Ae*xe_f(:, k) + Be*uk_f + cke;
    
    % controllo ottimo su tutto l'orizzonte temporale
    U_star(k,:) = uk_f';
end

figure(1)
hold on
plot(t, xk_f);
plot(t, x_des);
grid on;
legend("x_{mpc}", "y_{mpc}", "theta_{mpc}", "x_{des}", "y_{des}", "theta_{des}");


figure(2)
plot(t, U_star(1:max_iterations, :));
grid on
legend("u_{mpc}");

figure(3)
hold on
plot(xk_f(1, :), xk_f(2, :));
plot(x_des(1, :),x_des(2, :));
grid on
legend("MPC trajectory","desired trajectory", "Nu = 70")



%% ---- ENERGIA DEL CONTROLLO---------  %%
% si vuole minimizzare l'energia del controllo del sistema per portarlo
% sulla traiettoria desiderata.

xk_f = zeros(3, max_iterations);
x0 = [1;1;1.1];
xk_f(:, 1) = x0;
[Ac, Bc, Pc] = LPV_MPC_System(xk_f(:,1),[0;0]);
Q = eye(3);
R = eye(size(Bc, 2));
uk_f = zeros(max_iterations, 1);
A = Ac;
B = Bc;

Nu = 10; % numero di passi di predizione;
C = eye(3);
% periodo di simulazione
for k = 1:max_iterations-1
    x0 = xk_f(:, k);
    %keyboard;
    U_star = LPV_MPC_Controller(x0, Nu, A, B, C, Q, R);
    uk_f(k:k+size(B, 2)-1) = U_star(1:size(B,2),1);
    %disp(uk_f(k:k+size(B, 2)-1));
    [Ac, Bc, Pc] = LPV_MPC_System(x0,uk_f(k:k+size(B, 2)-1));
    A = (eye(size(Ac, 2)) + tau*Ac);
    B = tau*Bc;
    ck = tau*Pc;
    xk_f(:, k+1) = A*xk_f(:, k) + B*uk_f(k:k+size(B, 2)-1) + ck;
end


figure(1)
plot(t, xk_f);
grid on;
legend("x", "y", "theta");


figure(2)
plot(t, uk_f);
grid on
legend("u_{mpc}");


%%      MINIMIZZAZIONE DELL'ERRORE




