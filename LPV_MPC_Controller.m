function u = LPV_MPC_Controller(x, N, A, B, C, Q, R)
    u_min = -1.0;
    u_max = +1.0;
    y_min = -100.0;
    y_max = +100.0;

    R_bar = [];
    Q_bar = [];
    T_bar = A;
    S_bar = B;
    for i = 1:N
        R_bar = blkdiag(R_bar, R);
        Q_bar = blkdiag(Q_bar, Q);
        if (i < N)
            T_bar = [T_bar; T_bar(N-1)*A];
            S_bar = [S_bar; A*S_bar(N-1)];
        end
    end

    disp(S_bar)

    u = [1.0; 0.1];
end