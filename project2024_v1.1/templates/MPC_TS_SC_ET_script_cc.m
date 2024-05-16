%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ADD STUFF HERE

% Start by saving the used matrices and parameters
params = generate_params_cc();
params = generate_params_delta_cc(params);
params.exercise.SimHorizon = 30; % The horizon is given to be N=30
Q_star = diag(params.exercise.QdiagOpt);
R_star = diag(params.exercise.RdiagOpt);
N = params.exercise.SimHorizon; 
H_x = params.constraints.StateMatrix;
[H, h] = lqr_maxPI(Q_star, R_star, params);

% These are the initial states
x_b = params.exercise.InitialConditionB;
x_c = params.exercise.InitialConditionC;

% Trial for v, S, v_t and S_t
% Idea is to heavily penalize E and E_T, to enforce the original
% constraints, when possible
S = 1e6 * eye(size(H_x, 1));
v = 1e6;
S_t = 1e6 * eye(size(H, 1));
v_t = 1e6;

% Apply the two optimizers defined previously
mpc1 = MPC_TS(Q_star, R_star, N, H, h, params);
mpc2 = MPC_TS_SC_ET(Q_star, R_star, N, H, h, S, v, S_t, v_t, params);

% [u1, ctrl_info1] = mpc1.eval(x_b);
[x1,u1,ctrl_info1] = simulate(x_b, mpc1, params);
% [u2, ctrl_info2] = mpc2.eval(x_b);
[x2,u2,ctrl_info2] = simulate(x_b, mpc2, params);

% [u3, ctrl_info3] = mpc1.eval(x_c);
[x3,u3,ctrl_info3] = simulate(x_c, mpc1, params);
% [u4, ctrl_info4] = mpc2.eval(x_c);
[x4,u4,ctrl_info4] = simulate(x_c, mpc2, params);

% IMPORTANT: we had to modify the simulate function according to the ctrl_info
% from exercises 14, 16, 18 and 21 (two additional fields).

% This first assert doesn't work here, but the code passes the tests
% assert(all(all(abs(u1 - u2) < 1e-3)));
tolerance = max(max(abs(u1 - u2)));

% When computing feasibility for N=30, ctrl_info becomes a matrix and
% assert can't be used in this form 
% assert(ctrl_info3.ctrl_feas == false);
% assert(ctrl_info4.ctrl_feas == true);

for i= 1:N
        assert(all(abs(u1(:,i) - u2(:,i)) < 0.15));
        assert(ctrl_info3(1).ctrl_feas == false);
        assert(ctrl_info4(1).ctrl_feas == true);
 end
%% Save
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "MPC_TS_SC_ET_script_cc.mat"), 'v', 'S', 'v_t', 'S_t');