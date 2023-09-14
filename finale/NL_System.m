function dxdt = NL_System(x, u_mpc)
    v = u_mpc(1);
    omega = u_mpc(2);

    dx1dt = v * cos(x(3));  % dx/dt = v * cos(theta)
    dx2dt = v * sin(x(3));  % dy/dt = v * sin(theta)
    dx3dt = omega;          % dtheta/dt = omega

    dxdt = [dx1dt; dx2dt; dx3dt];
end