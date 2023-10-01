function x_des = genTrajectory(n, max_iterations, tipo_traiettoria, ngiri, center, r)

x_des = zeros(n,max_iterations);

switch tipo_traiettoria
    case 1     % Ellisse
        a = 2; % Semiasse maggiore
        b = 1; % Semiasse minore
        theta = linspace(0, ngiri*2*pi, max_iterations);
        for i = 1:max_iterations
            x_des(1, i) = center(1, 1) + a*cos(theta(i));
            x_des(2, i) = center(2, 1) + b*sin(theta(i));
            x_des(3, i) = theta(i);
        end
        title_text = 'Ellisse';

    case 2 % Circonferenza
        theta = linspace(0, ngiri*2*pi, max_iterations);
        for i = 1:max_iterations
            x_des(1, i) = center(1, 1) + r*cos(theta(i));
            x_des(2, i) = center(2, 1) + r*sin(theta(i));
            x_des(3, i) = theta(i) + pi/2;
        end
        title_text = 'Circonferenza';

    case 3 % Parabola
        a = 0.5; % Fattore di apertura
        x_des(1,:) = linspace(0, ngiri*2*pi, max_iterations);
        x_des(2,:) = a*x_des(1, :).^2;
        x_des(3,:) = atan2(2*a*x_des(1, :), 1);
        title_text = 'Parabola';

    case 4 % Sinusoide
        A = 1;   % Amplitude
        w = 2;   % Frequency
        phi = 0; % Phase offset
        x_des(1, :) = linspace(0, ngiri*2*pi, max_iterations);
        x_des(2, :) = A*sin(w * x_des(1, :) + phi);
        % Calcolo dell'orientamento θ per la sinusoide
        dx = [diff(x_des(1, :)), 1];
        dy = [diff(x_des(2, :)), 1];
        x_des(3, :) = atan2(dy, dx);
        title_text = 'Sinusoide';

    case 5 % Onda quadra
        A = 1; % Amplitude
        T = 4; % Period
        x_des(1,:) = linspace(0, ngiri*2*pi, max_iterations);
        x_des(2,:) = A*square(2 * pi * x_des(1, :) / T);
        % Calcolo dell'orientamento θ per l'onda quadra
        dx = [diff(x_des(1, :)), 1];
        dy = [diff(x_des(2, :)), 1];
        x_des(3, :) = atan2(dy, dx);
        title_text = 'Onda Quadra';

    case 6 % Spirale
        A = 0.3; % Amplitude
        w = 0.6; % Angular frequency
        theta = linspace(0, ngiri*2*pi, max_iterations);
        r = A*theta;
        x_des(1, :) = r .* cos(theta);
        x_des(2, :) = r .* sin(theta);
        % Calcolo dell'orientamento θ per la spirale
        dx = [diff(x_des(1,:)), 1];
        dy = [diff(x_des(2,:)), 1];
        x_des(3,:) = theta;
        title_text = 'Spirale';

    case 7 % Lineare
        a = 0.5; % Coefficiente angolare
        x_des(1,:) = linspace(0, ngiri*2*pi, max_iterations);
        x_des(2,:) = linspace(0, ngiri*2*pi, max_iterations);
        x_des(3,:) = atan2(a, 1);
        title_text = 'Lineare';

    otherwise
        disp('Tipo di traiettoria non valido');
        return;
end
end
