%% Ex1 Linearize SIMO System 
    
% Define Symbolic system variables and apply system model
    % Define symbolic system variables
        % Input variables
            syms phiw_dot_ref
        % State variables
            syms p p_dot theta theta_dot
        % Constants
            syms r l g

    % Define sonnection between symbolic variables. Here, we can use the
    % system description in the external .m-file again
        [p,theta,...                        % Sytem outputs
        d_p,dd_p,d_theta,dd_theta] = ...    % State updates
        Sigi_SISO_NonlinearODE(...
        phiw_dot_ref,...                               % Sytem input
        p,p_dot,theta,theta_dot,...         % System states
        l, r, g);                        % System parameters
    
    
% Calculate symbolic linearized state-space description
    % State space description:  x_dot = Ax + Bu
    %                           y     = Cx + Du

    % State-space matrix A
        A_symb = simplify(jacobian([d_p,dd_p,d_theta,dd_theta],[p,p_dot,theta,theta_dot]));
    % State-space matrix B
        B_symb = simplify(jacobian([d_p,dd_p,d_theta,dd_theta],phiw_dot_ref));
    % State-space matrix C
        C_symb = simplify(jacobian([p,theta],[p,p_dot,theta,theta_dot]));
    % State-space matrix D
        D_symb = simplify(jacobian([p,theta],phiw_dot_ref));

% Symbolic expression of state matrices in linearization point
    % Define linearization point
        phi_w       = 0;
        phi_w_dot   = 0;
        theta       = 0;
        theta_dot   = 0;

    % Evaluate state description as a function of system constants 
        A_symb_eq = eval(subs(A_symb)); 
        B_symb_eq = eval(subs(B_symb)); 
        C_symb_eq = eval(subs(C_symb)); 
        D_symb_eq = eval(subs(D_symb)); 

% Numerical expression of state matrices in linearization point
    %  Set values of system contstants        
        r  = par.r;
        l  = par.l;
        g  = par.g;

	% Evaluate symbolic state matrices numerically    
        A = eval(subs(A_symb_eq)); 
        B = eval(subs(B_symb_eq)); 
        C = eval(subs(C_symb_eq)); 
        D = eval(subs(D_symb_eq)); 
        
%% Design controller

switch SetParams
    case 'sigi_v1'
        % Make LQR to simply keep segway vertical
            Q = diag([1e3 0.1 1 0.1]);  % weighting matrix Q - penalize states [phiw, dphiw, theta, dtheta]
            R = 1e-1;                   % weighting matrix R 
            K_lqr = lqr(A,B,Q,R)       % LQR gain matrix 
    case 'sigi_v2'
        % Make LQR to simply keep segway vertical
%             Q = diag([5e2 0.1 10 0.1]);  % weighting matrix Q - penalize states [phiw, dphiw, theta, dtheta]
            Q = diag([5e2 0.1 0 10]);  % weighting matrix Q - penalize states [phiw, dphiw, theta, dtheta]
            R = 1e-1;                   % weighting matrix R 
            K_lqr = lqr(A,B,Q,R);       % LQR gain matrix 
end
