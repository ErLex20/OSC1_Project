function [A, B, P] = LPV_MPC_System(x_eq, u_eq)
    theta_eq = x_eq(3);
    v_eq = u_eq(1);
    omega_eq = u_eq(2);

    A = [0, 0, -v_eq * sin(theta_eq);
         0, 0, +v_eq * cos(theta_eq);
         0, 0, 0];

    B = [cos(theta_eq), 0;
         sin(theta_eq), 0;
         0, 1];

%     B = [cos(theta_eq), 0;
%          sin(theta_eq), 0;
%          0, 1];

    P = [v_eq * cos(theta_eq);
         v_eq * sin(theta_eq);
         omega_eq];
end
