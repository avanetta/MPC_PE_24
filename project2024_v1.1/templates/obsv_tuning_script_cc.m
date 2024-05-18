%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ADD STUFF HERE

poles = [0.9, 0.9, 0.5, 0.75, 0.5, 0.9];
params = generate_params_cc();
parmas_aug = generate_params_aug_obs(params);
Nsim = params.exercise.SimHorizon;
x0 = params.exercise.InitialConditionA;
alpha_o = [params.model.a1o; params.model.a2o; params.model.a3o];
eta_A = params.exercise.RadiationB;

d = alpha_o * params.exercise.To + eta_A;

%u = [-1000; -1000];

L = compute_observer_gain(poles, parmas_aug);
obsv = Linear_Observer(L, parmas_aug);
ctrl = Controller_constant;
[u, ctrl_info] = ctrl.eval;
if isscalar(u)
u = [1 ; 1]*u;
end
nx = params.model.nx;
nd = params.model.nd;
nu = params.model.nu;
X = zeros(nx,Nsim + 1);
X_est_tilde = zeros(nx+nd,Nsim + 1);

X(:,1) = x0;
X_est_tilde(:,1) = [x0;d];

for i= 1:Nsim
    X(:,i+1) = params.model.A*X(:,i) + params.model.B*u + params.model.Bd * d;
    X_est_tilde(:,i+1) = obsv.eval(X_est_tilde(:,i),u, params.model.C*X(:,i));
    %X_est(:,i+1) = parmas_aug.model.A*X_est(:,i) + [params.model.B;zeros(nd,nu)]*u + L*(parmas_aug.model.C*X_est(:,i) - params.model.C*X(:,i+1));
end

norm(X(:,2)-X_est_tilde(1:nx,2),2)
norm(X(:,11)-X_est_tilde(1:nx,11),2)
norm(X(:,31)-X_est_tilde(1:nx,31),2)

X_est = X_est_tilde(1:nx,:);
%% Save
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "obsv_tuning_script_cc.mat"), 'poles', 'X', 'X_est');
