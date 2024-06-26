%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [params] = generate_params_cc()
    params = struct();
    Tf = 60* 60; % = 60 minutes
    dt = 60;

    % model
    params.model = struct(...
        'nx', 3, ...
        'nu', 2, ...
        'nd', 3, ... % number of disturbances
        'ny', 3, ...
        'm1', 200*1000, ...
        'm2', 400*1000, ...
        'm3', 500*1000, ...
        'a1o', 0.1e2, ...
        'a2o', 1e2, ...
        'a3o', 3e2, ...
        'a12', 0.75e2, ...
        'a23', 2e2, ...
        'TimeStep', dt ...
    );

    % constraints
    params.constraints = struct(...
        'T1Max', -15, ...
        'T1Min', -1000, ...
        'T2Max', 4, ...
        'T2Min', 0, ...
        'T3Max', 1000, ...
        'T3Min', -1000, ...
        'P1Max', 0, ...
        'P2Max', 0, ...
        'P1Min', -2500, ...
        'P2Min', -2000);

    params.exercise = struct(...
        'SimHorizon', ceil(Tf / dt), ...
        'MPCHorizon', 60, ...
        'InitialConditionA', [-18; 1.3; 7.32], ...
        'InitialConditionB', [-22; 0; 2.82], ...
        'InitialConditionC', [12; 12; 12], ...
        'T_ref', [-21; 0.3], ...
        'To', 12, ...
        'QdiagOpt', [1, 1, 0], ...
        'RdiagOpt', [2.4, 0.5] * 1e-5, ...
        'x0_est', zeros(params.model.nx, 1), ...
        'd0_est', zeros(params.model.nd, 1), ...
        'RadiationA', zeros(3,1), ...
        'RadiationB', [200; 100; 0] ...
         );

    params.model.C = eye(params.model.ny, params.model.nx);
    params.model.Cd = zeros(params.model.nx, params.model.nd);
    params.model.C_ref = eye(size(params.exercise.T_ref,1), params.model.nx);
    params.model.D = zeros(params.model.nx, params.model.nu);

    % CONTINUE BELOW THIS LINE

    % Ex 4

    % AV Get discretized system matrices into params struct
    [Ac, Bc, Bd_c] = generate_system_cont_cc(params);
    [A, B, Bd] = discretize_system_dist(Ac, Bc, Bd_c, params);
    params.model.A = A;
    params.model.B = B;
    params.model.Bd = Bd;
    % AV Could also be done by this but uglier
    % [params.model.A, params.model.B, params.model.Bd] = discretize_system_dist(Ac, Bc, Bd_c, params);
    
    % AV Get constraint matrices & vectors into params struct
    [H_u, h_u, H_x, h_x] = generate_constraints_cc(params);
    params.constraints.InputMatrix = H_u;
    params.constraints.InputRHS = h_u;
    params.constraints.StateMatrix = H_x;
    params.constraints.StateRHS = h_x;
    % AV Could also be done by this but uglier
    % [params.constraints.InputMatrix, params.constraints.InputRHS, params.constraints.StateMatrix, params.constraints.StateRHS] = generate_constraints_cc(params);
    
    

    
end
