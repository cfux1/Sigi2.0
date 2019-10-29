function params = parameter()

%% Definition of physical parameters used for the model

% As reference the following source code hase been taken: https://..
% github.com/calm0815/BalboaRobotics/tree/master/Balboa32U4_MATLAB

%% Pendelum

params.m_p = 0.2913 / 2;                           % mass in kg
% I_p has to be adjusted due to addition of raspberry pi
params.I_p = 0.000229712 / 2;                      % moment of inertia in kg*m^2
params.r_p = 0.014264;                             % distance to center of mass in m

%% Wheel 

% Defined for each wheel individual

params.m_w = 0.0197;                               % mass in kg
params.I_w = 0.000023;                             % moment of inertia in kg*m2
params.r_w = 0.040;                                % distance to center of mass in m
params.width = 0.107;                              % m

%% Motor

% Two motors adjusted to each wheel

params.k_t = 0.003382287;                          % N*m/A
params.k_b = 0.003067344;                          % V*sec/rad
params.n = 110.103;                                % 
params.t_m = 0.000472285;                          % N*m
params.R_m = params.n*params.k_t*params.k_b/params.t_m;                        %
params.R_driver = 0.280;                           %
params.R = params.R_m + params.R_driver;                         %
params.m_rotator = 0.002;                          % mass of motor in kg
params.r_rotator = 0.004;                          % distance to center of mass in m
params.I_m = 0.5*params.m_rotator*params.r_rotator^2;            % moment of inertia in kg*m2
params.V_offset = params.R*params.t_m/params.k_t;                       % offset Voltage

%% Physical constants 

params.g = 9.81;                                   % m/s^2
end