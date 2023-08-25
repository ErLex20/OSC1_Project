function u_opt = LPV_MPC_Controller(x, N, A, B, C, Q, R)
    u_min = -1.0;
    u_max = +1.0;
    y_min = -100.0;
    y_max = +100.0;

    R_bar = [];
    Q_bar = [];
    T_bar = A;
    X = A;
    P_a = B;
    P = B;

    for i = 1:1:N
        R_bar = blkdiag(R_bar, R);
        Q_bar = blkdiag(Q_bar, Q);
        if (i > 1)
            X = X*A;
            T_bar = [T_bar; X];
            P_a = A*P_a;
            P = [P; P_a];
        end
        
    end
    
    S_bar = P;
    c = 1;
    for i = size(B, 2) + 1 : size(B, 2) : size(B, 2)*(N-1) + 1
        S_bar(:, i:i+size(B, 2)-1) = [zeros(size(A,2)*c, size(B,2)); S_bar(1:size(A, 2)*N-size(A, 2)*c, 1:size(B,2))];
        c = c + 1;
    end

    Y = 2*(Q + T_bar' * Q_bar * T_bar);
    H = 2*(R_bar + S_bar' * Q_bar * S_bar);
    F = 2*(T_bar'*Q_bar*S_bar)';
    init = zeros(size(H, 2), 1);
    
    
    objective = @(u) 0.5*u'*H*u + x'*F'*u;      %funzione obiettivo
    % aggiungere i vincoli possibili sulla u tramite le matrici G, W, E
    
    options = optimoptions('fmincon', 'Display', 'iter');  % Opzioni di ottimizzazione

    u = fmincon(objective, init, [], [], [], [], [], [], @mycon, options);
    %u = fmincon(objective, init, [],[]);

    %ustar = fmincon(@(z) 0.5*z'*H*z+x'*F'*z, init, [],[],[],[],[],[],@mycon)
    %u = -inv(H)*F*x;
    u_opt = [u(1); u(2)];
end

function [c, ceq] = mycon(u)
    c = [];    % Vincoli di disuguaglianza (vuoti nel tuo caso)
    ceq = [];  % Vincoli di uguaglianza (vuoti nel tuo caso)
end



