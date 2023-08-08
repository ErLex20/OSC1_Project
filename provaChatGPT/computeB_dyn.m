% Funzione per calcolare la matrice B in base all'orientamento corrente
function B_dyn = computeB_dyn(theta)
    B_dyn = [0;
             0;
             cos(theta)];
end