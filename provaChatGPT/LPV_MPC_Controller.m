function u = LPV_MPC_Controller(x, u_eq, A, B, N, Q, R)
    % Implementazione del controller LPV-MPC.

    % Definizione delle dimensioni del sistema
    nx = size(A, 1);
    nu = size(B, 2);

    % Inizializzazione delle matrici per il problema di ottimizzazione quadra
    H = zeros(N*nu, N*nu);
    f = zeros(N*nu, 1);
    Aeq = zeros(N*nx, N*nx);
    beq = zeros(N*nx, 1);

    % Costruzione delle matrici per il problema di ottimizzazione quadra
    for i = 1:N
        Ai = A - computeB_dyn(x(3)) * u_eq(2);
        Bi = B;
        
        H((i-1)*nu+1:i*nu, (i-1)*nu+1:i*nu) = R;
        f((i-1)*nu+1:i*nu) = -R * u_eq;
        
        if i == 1
            Aeq(1:nx, 1:nu) = -eye(nx);
            beq(1:nx) = Ai * x + Bi * u_eq - x;
        else
            Aeq((i-1)*nx+1:i*nx, (i-2)*nu+1:(i-1)*nu) = Ai;
            Aeq((i-1)*nx+1:i*nx, (i-1)*nu+1:i*nu) = -eye(nx);
            beq((i-1)*nx+1:i*nx) = Ai * x + Bi * u_eq;
        end
    end

    % Risoluzione del problema di ottimizzazione quadra
    options = optimset('Display', 'off');
    u_opt = quadprog(H, f, [], [], Aeq, beq, [], [], [], options);

    % Estrazione dell'azione di controllo ottimale
    u = u_opt(1:nu);
end
