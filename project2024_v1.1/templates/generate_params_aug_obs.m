%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [params_aug_obs] = generate_params_aug_obs(params)
    A = params.model.A;
    B = params.model.B;
    Bd = params.model.Bd;
    Cd = params.model.Cd;
    C = params.model.C;
    nx = params.model.nx;
    nd = params.model.nd;
    H_x = params.constraints.StateMatrix;
    % h_x = params.constraints.StateRHS;

    A_aug = [A, Bd; zeros(nd, nx), eye(nd, nd)];
    B_aug = [B; zeros(nd, size(B, 2))]; 
    C_aug = [C, Cd];
    nx_aug = nx + nd;
    H_x_aug = [H_x, zeros(size(H_x, 1), nd)];
    % h_x_aug = h_x;

    
    % params.constraints.StateRHS = h_x_aug;

    % This should be done at the end
    params_aug_obs = params;

    params_aug_obs.model.A = A_aug;
    params_aug_obs.model.B = B_aug;
    params_aug_obs.model.C = C_aug;
    params_aug_obs.model.nx = nx_aug;
    params_aug_obs.constraints.StateMatrix = H_x_aug;

end