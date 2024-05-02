%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% CONTINUE BELOW THIS LINE


params = generate_params_cc();
params_delta = generate_params_delta_cc(params);

Q = [5300, 0, 0;
    0, 5700, 0;
    0, 0, 0];
R = [0.08,0;
    0,0.008];
ctrl = LQR(Q,R,params_delta);

[X, U, ctrl_info] = simulate(params_delta.exercise.InitialConditionA, ctrl, params_delta);
[X_abs, U_abs] = traj_delta2abs(X,U,params_delta.exercise.x_s,params_delta.exercise.u_s);
[T1_max, T2_min, T2_max, P1_min, P1_max, P2_min, P2_max, input_cost, cstr_viol] = traj_constraints_cc(X_abs,U_abs,params_delta);

assert(cstr_viol == false);
assert(abs(X(1,30)) <= 3e-1);
assert(abs(X(2,30)) <= 2e-2);
assert(abs(X(1,60)) <= 3e-2);
assert(abs(X(2,60)) <= 2e-3);


%% Save results
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "lqr_tuning_script_cc.mat"), 'Q', 'R', 'X', 'U');

