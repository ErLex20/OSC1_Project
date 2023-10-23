function [theta_des, ngiri] = CorrectionTrajectory(x, y, theta_prev, ngiri)
    
    theta_des = atan2(y, x) + ngiri*2*pi;
    if(abs(theta_des + 2*pi - theta_prev) < abs(theta_des - theta_prev))
        theta_des = theta_des + 2*pi;
        ngiri = ngiri + 1;
    else
        if(abs(theta_des - 2*pi -theta_prev) < abs(theta_des - theta_prev))
            theta_des = theta_des - 2*pi;
            ngiri = ngiri - 1;
        end
    end
end