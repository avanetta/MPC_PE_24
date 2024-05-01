%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [T1_max, T2_min, T2_max, P1_min, P1_max, P2_min, P2_max, input_cost, cstr_viol] = traj_constraints_cc(X, U, params)
    T1_max = max(X(1,:));
    T2_min = min(X(2,:));
    T2_max = max(X(2,:));
    P1_min = min(U(1,:));
    P1_max = max(U(1,:));
    P2_min = min(U(2,:));
    P2_max = max(U(2,:));
    input_cost = 0;
    Nsim = size(U,2);
    for i = 1:Nsim
        %No Matrix R according to instructions?
        input_cost = input_cost + U(:,i)'*U(:,i);
    end
    cstr_viol = ~(T1_max <= params.constraints.T1Max && T2_min >= params.constraints.T2Min && ...
        T2_max <= params.constraints.T2Max && P1_min >= params.constraints.P1Min && ...
        P1_max <= params.constraints.P1Max && P2_min >= params.constraints.P2Min && ...
        P2_max <= params.constraints.P2Max);
end

