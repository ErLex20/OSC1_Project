function delta_xk_next = LPV_MPC_Simulation(delta_xk, delta_uk, dt, A, B, P)
    delta_xk_next = (A * dt + eye(size(A,1))) * delta_xk + B * dt * delta_uk + P * dt;
end