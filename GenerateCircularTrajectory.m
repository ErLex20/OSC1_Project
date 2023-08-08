function [x, y] = GenerateCircularTrajectory(R)
    % Genera punti lungo il cerchio
    num_points = 100;
    t = linspace(0, 2*pi, num_points);
    x = R * cos(t);
    y = R * sin(t);
end