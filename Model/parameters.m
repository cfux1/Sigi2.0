%% Definition of physical parameters used for the model

% As reference the following source code hase been taken: https://..
% github.com/calm0815/BalboaRobotics/tree/master/Balboa32U4_MATLAB

%% Pendelum

m_p = 0.2913 / 2;                           % mass in kg
% I_p has to be adjusted due to addition of raspberry pi
I_p = 0.000229712 / 2;                      % moment of inertia in kg*m^2
r_p = 0.014264;                             % distance to center of mass in m

%% Wheel 

% Defined for each wheel individual

m_w = 0.0197;                               % mass in kg
I_w = 0.000023;                             % moment of inertia in kg*m2
r_w = 0.040;                                % distance to center of mass in m
width = 0.107;                              % m

%% Motor

% Two motors adjusted to each wheel

k_t = 0.003382287;                          % N*m/A
k_b = 0.003067344;                          % V*sec/rad
n = 110.103;                                % 
t_m = 0.000472285;                          % N*m
R_m = n*k_t*k_b/t_m;                        %
R_driver = 0.280;                           %
R = R_m + R_driver;                         %
m_rotator = 0.002;                          % mass of motor in kg
r_rotator = 0.004;                          % distance to center of mass in m
I_m = 0.5*m_rotator*r_rotator^2;            % moment of inertia in kg*m2
V_offset = R*t_m/k_t;                       % offset Voltage

%% Physical constants 

g = 9.81;                                   % m/s^2