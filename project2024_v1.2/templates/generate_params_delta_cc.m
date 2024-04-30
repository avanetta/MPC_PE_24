%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params_delta = generate_params_delta_cc(params)
    
params_delta = params;

params_delta.model = rmfield(params_delta.model, 'C');
params_delta.model = rmfield(params_delta.model, 'Cd');
params_delta.model = rmfield(params_delta.model, 'C_ref');
params_delta.model = rmfield(params_delta.model, 'D');
params_delta.model = rmfield(params_delta.model, 'Bd');

% CONTINUE BELOW THIS LINE

% Exercise 6

% Calculate d and compute steady-state x_s, u_s
alpha_o = [params.model.a1o; params.model.a2o; params.model.a3o];
eta_A = params.exercise.RadiationA;

d = alpha_o * params.exercise.To + eta_A;

[x_s, u_s] = compute_steady_state(params, d);

% add steady-state to params_delta
params_delta.exercise.x_s = x_s;
params_delta.exercise.u_s = u_s;

% adapt intial conditions according dx_0 = x_0 - x_s
params_delta.exercise.InitialConditionA = params.exercise.InitialConditionA - x_s;
params_delta.exercise.InitialConditionA = params.exercise.InitialConditionB - x_s;
params_delta.exercise.InitialConditionA = params.exercise.InitialConditionC - x_s;

% adapt constraints according to 
% Hx * x <= hx
% --> Hx * (dx + xs) <= hx
% --> Hx * dx <= hx - Hx * xs
% --> dHx = Hx and dhx = hx - Hx * xs
% analogously for dHu and dhu
params_delta.constraints.StateRHS = params.constraints.StateRHS - params.constraints.StateMatrix * x_s;
params_delta.constraints.InputRHS = params.constraints.InputRHS - params.constraints.InputMatrix * u_s;


end