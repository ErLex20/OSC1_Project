function [x, y, theta] = GenerateTrajectory(trajectory, num_points)
    if (strcmp(trajectory, "Circular"))
        R = 5;
        t = linspace(0, 2*pi, num_points);
        x = R * cos(t);
        y = R * sin(t);
        dx = -R * sin(t);
        dy = R * cos(t);
        for i = 1:num_points
            theta(i) = atan2(dy(i), dx(i));
        end
    end
    
    if (strcmp(trajectory, "Straight"))
        L = 100;
        x = linspace(0, L, num_points);
        y = linspace(0, L, num_points);
        theta = atan2(y, x);
    end

    if (strcmp(trajectory, "Sinusoidal"))
        amplitude = 5;
        omega = 1;
        phi = 0;
        t = linspace(0, 2*pi, num_points);
        x = zeros(1, num_points);
        y = amplitude * sin(omega * t + phi);
        theta = omega * cos(omega * t + phi);
    end
end