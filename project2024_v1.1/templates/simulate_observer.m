%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X, U, X_est, D_est, ctrl_info] = simulate_observer(x0, x0_est, d0_est, ctrl, obsv, Disturbances, params)
    % x0 is the true initial condition
    % x0_est is the estimated initial condition
    % d0_est is the estimated disturbance
    % ctrl is the controller object
    % obsv is the observer object (task 26)
    % Disturbances is an array of disturbances (1x(N_sim-1))
    % params describes the params struct, not params_augmented

    % FOR THIS EXERCISE, I STARTED BY USING THE simulate.m AS STARTING
    % POINT FOR THE OVERALL STRUCTURE:
    %{
    % Get the needed parameters from the struct
    nx = params.model.nx;
    nu = params.model.nu;
    Nsim = params.exercise.SimHorizon;
    
    % Initialize x and u with correct length
    x = zeros(nx,Nsim+1);
    u = zeros(nu, Nsim);
    
    ctrl_info = struct('ctrl_feas', cell(1,Nsim));
   
    x(:,1) = x0;

    % Iterative step
    for i= 1:Nsim
        % Get control input
        [u(:,i), ctrl_info(i)] = ctrl.eval(x(:,i));
        % Simulate system
        x(:,i+1) = params.model.A*x(:,i) + params.model.B*u(:,i);
    end
    %}
    nx = params.model.nx;
    nd = params.model.nd;
    nu = params.model.nu;
    Nsim = params.exercise.SimHorizon;
    U = zeros(nu, Nsim);
    X(:,1) = x0;
    X_est(:,1) = x0_est;
    D_est(:,1) = d0_est;
    params_aug = generate_params_aug_obs(params);
    for i= 1:Nsim
        %a
        [x_s,u_s] = compute_steady_state(params,D_est(:,i));
        %b
        [U(:,i), ctrl_info(i)] = ctrl.eval(X_est(:,i), D_est(:,i), x_s, u_s);
        %c
        X(:,i+1) = params.model.A*X(:,i) + params.model.B*U(:,i) + params.model.Bd*Disturbances(:,i);
        %d
        x0_est_tilde = [X(:,i);Disturbances(:,i)];
        x0_est_pred_tilde = obsv.eval([X_est(:,i);D_est(:,i)], U(:,i), params_aug.model.C*x0_est_tilde);
        X_est(:,i+1) = x0_est_pred_tilde(1:nx);
        D_est(:,i+1) = x0_est_pred_tilde(nx+1:nx+nd);
    end
end
