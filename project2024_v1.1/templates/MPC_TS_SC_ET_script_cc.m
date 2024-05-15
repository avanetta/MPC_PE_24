%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ADD STUFF HERE
% Between this two lines: Arnaud's code

% Start by saving the used matrices and parameters
params = generate_params_cc(); % Or should we use generate_params_delta_cc here?
Q_star = diag(params.exercise.QdiagOpt);
R_star = diag(params.exercise.RdiagOpt);
N = params.exercise.SimHorizon/2; % The horizon is given to be N=30
H_x = params.constraints.StateMatrix;

% These are the initial states
x_b = params.exercise.InitialConditionB;
x_c = params.exercise.InitialConditionC;

% Trial for v, S, v_t and S_t
% Idea is to heavily penalize E and E_T, to enforce the original
% constraints, when possible
S = 1e6 * eye(size(H_x, 1));
v = 1e6;
St = 1e6 * eye(size(H, 1));
vt = 1e6;

% Apply the two optimizers defined previously
mpc1 = MPC_TS(Q_star, R_star, N, H, h, params);
mpc2 = MPC_TS_SC_ET(Q_star, R_star, N, H, h, S, v, S_t, v_t, params);

[u1, ctrl_info1] = mpc1.eval(x_b);
[u2, ctrl_info2] = mpc2.eval(x_b);

[u3, ctrl_info3] = mpc1.eval(x_c);
[u4, ctrl_info4] = mpc2.eval(x_c);


% Between this two lines: Arnaud's code

%% Save
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "MPC_TS_SC_ET_script_cc.mat"), 'v', 'S', 'v_t', 'S_t');