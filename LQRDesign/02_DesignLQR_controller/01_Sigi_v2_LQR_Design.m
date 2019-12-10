%% Clean up Matlab

clearvars;  % Clear all variables
close all;  % Close all figures
clc;        % Clear text in command window

%% Run Simulation of Sigi SIMO System

% Set system parameters
SetParams = 'sigi_v2'; 
% OPTIONS: 'sigi_v1', 'sigi_v2'
switch SetParams
    case 'sigi_v1'
        par.m       = 0.933;                    % [kg]      Body part mass
        par.l       = 0.085749;                 % [m]       Position of COM
    case 'sigi_v2'
        % experimentell "gemessen" von Raffi
        par.m       = 0.412;               % [kg]      Body part mass
        par.l       = 0.029;               % [m]       Position of COM
     
end
        par.r       = 0.04;                     % [m]       Radius of wheel
        par.g       = 9.81;                     % [m/s^2]   Gravity constant  
% Set simulation parameters
    par.t_sim               = 5;            % [s] Simulation length 
    par.Ts                  = 1e-4;         % [s] Simulation sampling time
    par.simopt.Solver       = 'ode1';       %     Numerical solver

% Set initial conditions of simulation
    p_0         = 0;                        % [m]     Initial value of position
    d_p_0       = 0;                        % [m/s]   Initial value of velocity
    theta_0     = 45 /360*2*pi;             % [rad]   Initial value of angle
    d_theta_0   = 0;                        % [rad/s] Initial value angular velocity

% Run simulation and extract data    
    % Run simulation
    tic
        simOut = sim('Sigi_SIMO',par.simopt);
    toc
    % Extract relevant data
        data.t          = simOut.tout;
        data.p          = simOut.yout{1}.Values.Data;
        data.d_p        = simOut.yout{2}.Values.Data;
        data.theta      = simOut.yout{3}.Values.Data;
        data.d_theta    = simOut.yout{4}.Values.Data;
    % Store all data - might be useful at some point
        data.SimOut = simOut;
    
% Plot simulation results using dedicated plotting function
    data            = PlotSimulationResults(data,par);

%% Design LQR Controller
    Design_LQR_Fullstate

% Run simulation using Fullstate LQR controller

        % Set initial conditions of simulation
    p_0         = 0;                        % [m]     Initial value of position
    d_p_0       = 0;                        % [m/s]   Initial value of velocity
    theta_0     = -10 /360*2*pi;             % [rad]   Initial value of angle
    d_theta_0   = 0;                        % [rad/s] Initial value angular velocity

% Run simulation and extract data    
    % Run simulation
        simOut = sim('Sigi_SISO_LQR_Fullstate_stabilisiert',par.simopt);
    % Extract relevant data
        data.t          = simOut.tout;
        data.p          = simOut.yout{1}.Values.Data;
        data.d_p        = simOut.yout{2}.Values.Data;
        data.theta      = simOut.yout{3}.Values.Data;
        data.d_theta    = simOut.yout{4}.Values.Data;
    % Store all data - might be useful at some point
        data.SimOut = simOut;

% Plot simulation results using dedicated plotting function
    data            = PlotSimulationResults(data,par);
 
%% Design 3x3 LQR Controller
    Design_LQR_3x3
    
% Run simulation using 3x3 LQR controller

        % Set initial conditions of simulation
    p_0         = 0;                        % [m]     Initial value of position
    d_p_0       = 0;                        % [m/s]   Initial value of velocity
    theta_0     = -25 /360*2*pi;             % [rad]   Initial value of angle
    d_theta_0   = 0;                        % [rad/s] Initial value angular velocity

% Run simulation and extract data    
    % Run simulation
        simOut = sim('Sigi_SISO_LQR_3x3_stabilisiert',par.simopt);
    % Extract relevant data
        data.t          = simOut.tout;
        data.p          = simOut.yout{1}.Values.Data;
        data.d_p        = simOut.yout{2}.Values.Data;
        data.theta      = simOut.yout{3}.Values.Data;
        data.d_theta    = simOut.yout{4}.Values.Data;
    % Store all data - might be useful at some point
        data.SimOut = simOut;

% Plot simulation results using dedicated plotting function
    data            = PlotSimulationResults(data,par);

%% Linearize 3x3 stabilized Sigi

% Define Symbolic system variables and apply system model
    % Define symbolic system variables
        % Input variables
            syms theta_ref p p_dot theta theta_dot m r l g

    % Define sonnection between symbolic variables. Here, we can use the
    % system description in the external .m-file again
        [p,theta,...                            % Sytem outputs
        d_p,dd_p,d_theta,dd_theta] = ...        % State updates
        Sigi_SISO_LQR_3x3_stabilisiert_NonlinearODE(...
        theta_ref,...                           % Sytem input
        p,p_dot,theta,theta_dot,...             % System states
        l, r, g,...                             % System parameters  
        K_lqr(1), K_lqr(2), K_lqr(3));          % LQR controller gain                                 
    
% Calculate symbolic linearized state-space description
    % State space description:  x_dot = Ax + Bu
    %                           y     = Cx + Du

    % State-space matrix A
        A_symb = simplify(jacobian([d_p,dd_p,d_theta,dd_theta],[p,p_dot,theta,theta_dot]));
    % State-space matrix B
        B_symb = simplify(jacobian([d_p,dd_p,d_theta,dd_theta],theta_ref));
    % State-space matrix C
        C_symb = simplify(jacobian(p,[p,p_dot,theta,theta_dot]));
%     % State-space matrix D
%         D_symb = simplify(jacobian(p,[p,p_dot,theta,theta_dot]));
%         C_symb = simplify(jacobian(theta,[p,p_dot,theta,theta_dot]));
    % State-space matrix D
        D_symb = simplify(jacobian(p,theta_ref));
% Symbolic expression of state matrices in linearization point
    % Define linearization point
        p           = 0;
        theta       = 0;

    % Evaluate state description as a function of system constants 
        A_symb_eq_tmp = eval(subs(A_symb)); 
        B_symb_eq_tmp = eval(subs(B_symb)); 
        C_symb_eq_tmp = eval(subs(C_symb)); 
        D_symb_eq_tmp = eval(subs(D_symb)); 

        p_dot       = 0;
        theta_dot   = 0;

        A_symb_eq = eval(subs(A_symb_eq_tmp)); 
        B_symb_eq = eval(subs(B_symb_eq_tmp)); 
        C_symb_eq = eval(subs(C_symb_eq_tmp)); 
        D_symb_eq = eval(subs(D_symb_eq_tmp)); 

% Numerical expression of state matrices in linearization point
    %  Set values of system contstants        
        m  = par.m;
        r  = par.r;
        l  = par.l;
        g  = par.g;

	% Evaluate symbolic state matrices numerically    
        A = eval(subs(A_symb_eq)); 
        B = eval(subs(B_symb_eq)); 
        C = eval(subs(C_symb_eq)); 
        D = eval(subs(D_symb_eq)); 


% Calculate transfer function from state-space system description
        sys = zpk(ss(A,B,C,D));


% Design PID Controller for outer SISO loop if you like

figure;grid on;hold on;
nyquist(sys/3);
ylim([-2 2]);
xlim([-2 2]);