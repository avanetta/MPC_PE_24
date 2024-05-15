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
    C = params.model.C;
    nx = params.model.nx;
    nd = params.model.nd;
    H_x = params.constraints.StateMatrix;
    % h_x = params.constraints.StateRHS;

    A_aug = [A, zeros(nx, nd); zeros(nd, nx), Bd];
    B_aug = [B; zeros(nd, size(B, 2))]; 
    C_aug = [C, zeros(size(C, 1), nd)];
    nx_aug = nx + nd;
    H_x_aug = [H_x, zeros(size(H_x, 1), nd)];
    % h_x_aug = h_x;

    params.model.A = A_aug;
    params.model.B = B_aug;
    params.model.C = C_aug;
    params.model.nx = nx_aug;
    params.constraints.StateMatrix = H_x_aug;
    % params.constraints.StateRHS = h_x_aug;

    % This should be done at the end
    params_aug_obs = params;

end