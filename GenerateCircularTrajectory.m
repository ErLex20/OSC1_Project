function [x, y, theta] = GenerateCircularTrajectory(R)
    num_points = 100;
    t = linspace(0, 2*pi, num_points);
    x = R * cos(t);
    y = R * sin(t);
    theta = -cot(atan2(x, y));
end