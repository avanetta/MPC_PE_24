%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ad, Bd, Bd_d] = discretize_system_dist(Ac, Bc, Bd_c, params)
    % Output function can not be extracted from params (probably choice of
    % MPC-Team)
    % Cc = params.model.C; % since C_cont = C_discrete
    % Dc = params.model.D; % since D_cont = D_discrete
    % Cc_d = params.model.Cd; 
    % C and D are used to define a state space system !!!

    % Shortcut via Definiton of
    %{
    nu = params.model.nu;
    nx = params.model.nx;
    ny = params.model.ny;
    nd = params.model.nd;

    Cc = eye(ny,nx);
    Dc = zeros(ny, nu);
    Cc_d = zeros(ny, nd);
    %}
    % We augment the system to also include the discturbance in the input

    Bc_new = [Bc Bd_c];
    %Dc_new = [Dc Cc_d];
    %sys_c = ss(Ac,Bc_new,Cc,Dc_new);
    
    %sys_d = c2d(sys_c, params.model.TimeStep);

    %[Ad, Bd_new, Cd, Dd_new] = ssdata(sys_d);

    % Split the augmented input again in input and disturbance
    nu = params.model.nu;
    [Ad, Bd_new] = c2d(Ac,Bc_new, params.model.TimeStep);
    Bd = Bd_new(:,1:nu);
    Bd_d = Bd_new(:,nu+1:end);

    
    
    
    

end