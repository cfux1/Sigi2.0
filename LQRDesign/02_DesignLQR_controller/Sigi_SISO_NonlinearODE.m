function    [p,theta,...                        % Sytem outputs
            d_p,dd_p,d_theta,dd_theta] = ...    % State updates
            Sigi_SISO_NonlinearODE(...
            phiw_dot_ref,...                    % Sytem input
            p,p_dot,theta,theta_dot,...         % System states
            l, r, g)                            % System parameters

% Additional parameters of electric motor model
	tau_lp = 0.09;

% State updates
    d_p         = p_dot;
    d_theta     = theta_dot;
    dd_p        = (l*(phiw_dot_ref*r^2*cos(theta) - p_dot*r*cos(theta) - 2*l*p_dot + r^2*theta_dot*cos(theta) + 2*l*phiw_dot_ref*r + 2*l*r*theta_dot + r^2*tau_lp*theta_dot^2*sin(theta) + g*r*tau_lp*sin(theta)))/(tau_lp*(2*l^2 + 2*cos(theta)*l*r + r^2));
    dd_theta    = (p_dot*r - phiw_dot_ref*r^2 - r^2*theta_dot + l*p_dot*cos(theta) - l*phiw_dot_ref*r*cos(theta) - l*r*theta_dot*cos(theta) + g*l*tau_lp*sin(theta) + l*r*tau_lp*theta_dot^2*sin(theta))/(tau_lp*(2*l^2 + 2*cos(theta)*l*r + r^2));
 
% Outputs
    p           = p;
    theta       = theta; 

