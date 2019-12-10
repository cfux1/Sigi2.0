function    [p,theta,...                        % Sytem outputs
            d_p,dd_p,d_theta,dd_theta] = ...    % State updates
            Sigi_SIMO_NonlinearODE(...
            T,...                               % Sytem input
            p,p_dot,theta,theta_dot,...         % System states
            m, l, r, g)                         % System parameters


% State updates
    d_p         = p_dot;
    d_theta     = theta_dot;
    dd_p        = -(2*T*l + T*r*cos(theta) + 2*l^2*m*r*theta_dot^2*sin(theta) - g*l*m*r*cos(theta)*sin(theta))/(l*m*r*(cos(theta)^2 - 2));
    dd_theta    = (T*r + T*l*cos(theta) - g*l*m*r*sin(theta) + l^2*m*r*theta_dot^2*cos(theta)*sin(theta))/(l^2*m*r*(cos(theta)^2 - 2));

% Outputs
    p           = p;
    theta       = theta; 
