clc
clear all 
close all

% definisco la dinamica dell'uniciclo;

v = 0.1;            % m/s
omega = 0.1;          % rad/s
T_sim = 100;         % s
tau = 0.01;          % s
theta = [0:tau:2*pi];
len = size(theta, 2);
incr_t = T_sim/len;



for i = 1:len
    dxdt(:,i) = [v*cos(theta(i));
                 v*sin(theta(i));
                 omega];
    if i == 1
        t(i) = 0;
    else
        t(i) = t(i-1) + incr_t;
    end
end


%% ---- ENERGIA DEL CONTROLLO---------  %%
% si vuole minimizzare l'energia del controllo del sistema per portarlo
% sulla traiettoria desiderata.

xk_f = zeros(3, len);
x0 = [1;1;1.1];
xk_f(:, 1) = x0;
[Ac, Bc, Pc] = LPV_MPC_System(xk_f(:,1),[0;0]);
Q = eye(3);
R = eye(size(Bc, 2));
uk_f = zeros(len, 1);
A = Ac;
B = Bc;

Nu = 1; % numero di passi di predizione;
C = eye(3);
% periodo di simulazione
for k = 1:len-2
    x0 = xk_f(:, k);
    %keyboard;
    U_star = LPV_MPC_Controller(x0, Nu, A, B, C, Q, R);
    uk_f(k:k+size(B, 2)-1) = U_star(1:size(B,2),1);
    %disp(uk_f(k:k+size(B, 2)-1));
    [Ac, Bc, Pc] = LPV_MPC_System(xk_f(:,k),uk_f(k:k+size(B, 2)));
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

% definisco ora la traiettoria desiderata dell'uniciclo, ad esempio una
% circonferenza;
x_des = zeros(3, len);
r = 1;                          % Raggio della circonferenza
x_start = [1;0;0];
x_des(:, 1) = x_start;
for i = 2:len
    x_des(1, i) = r*cos(theta(i));
    x_des(2, i) = r*sin(theta(i));
    x_des(3, i) = theta(i);
end

figure(1)
plot(x_des(1, :),x_des(2, :));
grid on


% questa dinamica dovrà essere sottratta alla x linearizzata del sistema, e
% data in pasto al controllore del modello mpc

xk_f = zeros(3, len);
x0 = x_start;
xk_f(:, 1) = x0;
[Ac, Bc, Pc] = LPV_MPC_System(xk_f(:,1),[0;0]);
Q = eye(3);
R = eye(size(Bc, 2));
uk_f = zeros(len, 1);
A = Ac;
B = Bc;

Nu = 100; % numero di passi di predizione;
C = eye(3);
for k = 1:len-2
    x_error = xk_f(:, k) - x_des(:,k);
    x0 = x_error;
    %keyboard;
    U_star = LPV_MPC_Controller(x0, Nu, A, B, C, Q, R);
    uk_f(k:k+size(B, 2)-1) = U_star(1:size(B,2),1);
    %disp(uk_f(k:k+size(B, 2)-1));
    [Ac, Bc, Pc] = LPV_MPC_System(xk_f(:, k),uk_f(k:k+size(B, 2)));
    A = (eye(size(Ac, 2)) + tau*Ac);
    B = tau*Bc;
    ck = tau*Pc;
    xk_f(:, k+1) = A*xk_f(:, k) + B*uk_f(k:k+size(B, 2)-1) + ck;
end

figure(2)
plot(t, xk_f);
grid on;
legend("x_{mpc}", "y_{mpc}", "theta_{mpc}");


figure(3)
plot(t, uk_f);
grid on
legend("u_{mpc}");

figure(4)
plot(xk_f(1, :), xk_f(2, :));
grid on
legend("x_e")
