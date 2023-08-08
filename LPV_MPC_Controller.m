function u = LPV_MPC_Controller(x, N, A, B, C, Q, R)
    u_min = -1.0;
    u_max = +1.0;
    y_min = -100.0;
    y_max = +100.0;

    R_bar = [];
    Q_bar = [];
    T_bar = A;
    X = A;
    S_bar = B;
    for i = 1:1:N
        R_bar = blkdiag(R_bar, R);
        Q_bar = blkdiag(Q_bar, Q);
        if (i > 1)
            X = X*A;
            T_bar = [T_bar; X];
        end
    end

    disp(S_bar)

    u = [1.0; 0.1];
end