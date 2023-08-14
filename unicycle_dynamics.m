function dxdt = unicycle_dynamics(t, x, u)
    x_pos = x(1);
    y_pos = x(2);
    theta = x(3);
    v = u(1);
    w = u(2);
    dxdt = [v*cos(theta); v*sin(theta); w];
end