function    [p,theta,...                            % Sytem outputs
            d_p,dd_p,d_theta,dd_theta] = ...        % State updates
            Sigi_SISO_LQR_3x3_stabilisiert_NonlinearODE(...
            theta_ref,...                           % Sytem input
            p,p_dot,theta,theta_dot,...             % System states
            l, r, g,...
            K_lqr_1, K_lqr_2, K_lqr_3)                                % System parameters

% C_MIMO parameters

K_lqr = [K_lqr_1, K_lqr_2, K_lqr_3];

% Electric motor - additional parameters
    tau_lp = 0.0994;

% Electric motor reference speed
    phiw_dot_ref = -K_lqr*[p_dot,theta,theta_dot]' + theta_ref * K_lqr(2) ;

% State updates with adapted equations
    d_p         = p_dot;
    d_theta     = theta_dot;
    dd_p        = (l*(phiw_dot_ref*r^2*cos(theta) - p_dot*r*cos(theta) - 2*l*p_dot + r^2*theta_dot*cos(theta) + 2*l*phiw_dot_ref*r + 2*l*r*theta_dot + r^2*tau_lp*theta_dot^2*sin(theta) + g*r*tau_lp*sin(theta)))/(tau_lp*(2*l^2 + 2*cos(theta)*l*r + r^2));
    dd_theta    = (p_dot*r - phiw_dot_ref*r^2 - r^2*theta_dot + l*p_dot*cos(theta) - l*phiw_dot_ref*r*cos(theta) - l*r*theta_dot*cos(theta) + g*l*tau_lp*sin(theta) + l*r*tau_lp*theta_dot^2*sin(theta))/(tau_lp*(2*l^2 + 2*cos(theta)*l*r + r^2));
 
% Outputs
    p           = p;
    theta       = theta; 

