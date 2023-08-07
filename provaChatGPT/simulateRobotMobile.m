function x_next = simulateRobotMobile(x, u, dt, v_eq, theta_eq, L)
    % Simulazione del robot mobile nel piano utilizzando il modello linearizzato.

    % Calcolo del punto di equilibrio
    x_eq = 0;
    y_eq = 0;
    theta_eq = 0;

    % Calcolo delle variazioni delle variabili di stato
    delta_x = x(1);
    delta_y = x(2);
    delta_theta = x(3);

    % Calcolo delle nuove variabili di stato
    delta_x_next = delta_x + v_eq * cos(theta_eq) * delta_theta * dt;
    delta_y_next = delta_y + v_eq * sin(theta_eq) * delta_theta * dt;
    delta_theta_next = delta_theta + u(2) * dt;

    % Aggiornamento delle variabili di stato
    x_next = [delta_x_next; delta_y_next; delta_theta_next];
end