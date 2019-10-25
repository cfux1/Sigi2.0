%% Lagrange

% Using matlab to create the dynamics of our sigi 2.0 
% A lagrange attempt is chosen, as a source file the same link is used as 
% for the parameters

%% Initialize

clear; format compact 

%% Creation of symbolic parameters

syms m_w r_w theta_w dtheta_w ddtheta_w I_w % wheel
syms m_p r_p theta_p dtheta_p ddtheta_p I_p % pendulum
syms n I_m k_t k_b R A % motor/gear
syms theta_w dtheta_w ddtheta_w theta_p dtheta_p ddtheta_p % radian
syms f_w g % torque, gravity

%% Generalized Coordinates

% Our generalized coordinates are the angle of rotation done by the wheel
% q(1) = theta_w 
% and the difference in angle to the upright position 
% q(2) = theta_p


q   = [theta_w theta_p]';                   % generalized coordinates
dq  = [dtheta_w dtheta_p]';                 % first derrivative (actual output)
ddq = [ddtheta_w ddtheta_p]';               % second derrivative
f   = [f_w 0]';                             % external force vector

%% Generalized coordinates to system coordinates

% Using q = [0; 0] and wheel center of mass as origin

% wheel: center of mass
x_w = r_w*(q(1) + q(2));
y_w = 0;

dx_w = diff(x_w, q(1))*dq(1) + diff(x_w, q(2))*dq(2);
dy_w = diff(x_w, q(1))*dq(1) + diff(y_w, q(2))*dq(2);

% pendelum: center of mass
x_p = x_w + r_p*sin(q(2));
y_p = r_p*cos(q(2));

dx_p = diff(x_p, q(1))*dq(1) + diff(x_p, q(2))*dq(2);
dy_p = diff(x_p, q(1))*dq(1) + diff(y_p, q(2))*dq(2);

% force vector
f(1) = -n^2*I_m*ddq(1) - n^2*k_t*k_b*dq(1)/R + A;

%% Kinetic and potential energy

% Calculated for all subsystems individually

% Wheel
K_w1 = 0.5*m_w*dx_w^2 + 0.5*m_w*dy_w^2;         % Translational kinetic energy
K_w2 = 0.5*I_w*(dq(1) + dq(2))^2;               % Rotational kinetic energy
U_w = m_w*g*y_w;                                % Potential energy

% Pendelum
K_p1 = 0.5*m_p*dx_p^2 + 0.5*m_p*dy_p^2;         % Translational kinetic energy
K_p2 = 0.5*I_p*dq(2)^2;                         % Rotational kinetic energy
U_p = m_p*g*y_p;                                % Potential energy


%% Lagrange

% L = Total kinetic - total potantial energy
L = (K_w1 + K_w2 + K_p1 + K_p2) - (U_w + U_p);


%% Differential of Lagrange

% d/dt(dL/d(dq))) - (dL/dq) = f

dLq = zeros(legth(q),1);
ddLq = zeros(length(q),1);
eq = zeros(length(q),1);

N = length(q);
for i = 1:N
    dLq(i) = diff(L, dq(i));
    
    temp = 0;
    for j = 1:N
        temp = temp + diff(dLq(i),dq(j))*ddq(j) + diff(dLq(i),q(j))*dq(j);
    end
    ddLq(i) = temp;
    
    eq(i) = ddLq(i) - diff(L,q(i)) - f(i);
end

eq = simplify(eq');