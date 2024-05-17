%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ADD STUFF HERE

poles = [-0.0; -1.0; -1.2; -1.4; -1.6; -1.8];
params = generate_params_cc();
parmas_aug = generate_params_aug_obs(params);
x0 = params.exercise.InitialConditionA;
alpha_o = [params.model.a1o; params.model.a2o; params.model.a3o];
eta_A = params.exercise.RadiationB;

d = alpha_o * params.exercise.To + eta_A;

u = [-1000; -1000];

L = compute_observer_gain(poles, parmas_aug);
nx = params.model.nx;
nd = params.model.nd;
nu = params.model.nu;
X = zeros(nx,30);
X_est = zeros(nx+nd,30);

X(:,1) = x0;
Xest(:,1) = [x0;d];

for i= 1:30
    X(:,i+1) = params.model.A*X(:,i) + params.model.B*u + params.model.Bd * d;
    X_est(:,i+1) = parmas_aug.model.A*X_est(:,i) + [params.model.B;zeros(nd,nu)]*u + L*(parmas_aug.model.C*X_est(:,i) - params.model.C*X(:,i));
end

norm(X(:,1)-X_est(1:nx,1),2)
norm(X(:,10)-X_est(1:nx,10),2)
norm(X(:,30)-X_est(1:nx,30),2)


%% Save
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "obsv_tuning_script_cc.mat"), 'poles', 'X', 'X_est');
