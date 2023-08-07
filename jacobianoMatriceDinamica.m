function [A, B] = jacobianoMatriceDinamica(v_eq, theta_eq)

    A = [0, 0, -v_eq * cos(theta_eq);
         0, 0, -v_eq * sin(theta_eq);
         0, 0, 0];

    B = [1, 0;
         0, 0;
         0, 1];
end
