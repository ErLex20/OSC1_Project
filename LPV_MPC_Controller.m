function u = LPV_MPC_Controller(x, N, A, B, C, Q, R)
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
            P_a = A*P_a;
            T_bar = [T_bar; X];
            P = [P; P_a];
        end
        
    end
    
    % calcolo della S_bar
    
    S_bar = P;
    c = 1;
    for i = size(B, 2) + 1 : size(B, 2) : size(B, 2)*(N-1) + 1
        S_bar(:, i:i+size(B, 2)-1) = [zeros(3*c, 2); S_bar(1:size(A, 2)*N-size(A, 2)*c ,1:size(B,2))]
        c = c + 1;
    end



    disp("S_bar");
    disp(S_bar);
    disp("T_bar");
    disp(T_bar)
    disp("P");
    disp(P);

    u = [1.0; 0.1];
end