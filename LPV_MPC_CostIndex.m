function cost = LPV_MPC_CostIndex(Q, R, X, U)
    cost = zeros(1, size(X, 2));
    for i = 1:size(X, 2)
        cost(i) = X(:, i)' * Q * X(:, i) + U(:, i)' * R * U(:, i);
    end
end
