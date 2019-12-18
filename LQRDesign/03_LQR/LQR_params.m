%% Set required parameters for LQR control

% Run this script to load required parameters for LQR controll into
% workspace

K_lqr = [-70.7107 -83.3587 -91.2872 -13.2266];  % Enter calculated values for K_lqr
p_ref = 0;                                      % position reference
theta_ref = 9;                                  % equilibrium angle
max_velocity = 6000;                            % max velocity for motor input = 1