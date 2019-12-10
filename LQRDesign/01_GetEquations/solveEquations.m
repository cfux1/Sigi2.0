clear all;
close all;
clc;

%% Apply simplifications in Matlab

% Copy equations from Mathematica by hand and write as matlab code
syms T r l m g
syms theta theta_dot theta_ddot
syms p p_dot p_ddot

% Equations from ApplyLagrange.nb (Mathematica)
    eq1 = T/r+l*m*sin(theta)*theta_dot^2 == m*(l*cos(theta)*theta_ddot+p_ddot);
    eq2 = T+l*m*(2*l*theta_ddot + cos(theta)*p_ddot) == g*l*m*sin(theta);
% Get system of equations in latex syntax
    latex(eq1)
    latex(eq2)

% Solve equations for relevant second derivatives
    sol = solve([eq1,eq2],{theta_ddot,p_ddot});
% Get final expressions in in latex syntax
    latex(sol.p_ddot)
    latex(sol.theta_ddot)

%% Simplification for Model on real Hardware
% New equations (lowpass on speed)
syms phiw_ddot phiw_dot 
syms phiw_dot_ref
syms tau_lp

% New equation
    eq3 = phiw_ddot == 1/tau_lp*(phiw_dot_ref - phiw_dot);
    
% Replace phiw with theta and p
    % Replacements to make
        % p_ddot = (phiw_ddot + theta_ddot)*r;
        % p_dot/r-theta_dot = phiw_dot; % p_dot = (phiw_dot + theta_dot)*r;
    eq4 = subs(eq3, {phiw_ddot},{p_ddot/r-theta_ddot});
    eq5 = subs(eq4, {phiw_dot},{p_dot/r-theta_dot});
    
% Solve new set of equations    
    eq6 = solve(eq1,T)==solve(eq2,T);
    sol_hardware = solve([eq5,eq6],{theta_ddot,p_ddot});

% Get new set of equations
    p_ddot = simplify(sol_hardware.p_ddot)
    theta_ddot = simplify(sol_hardware.theta_ddot)

    latex(p_ddot)
    latex(theta_ddot)