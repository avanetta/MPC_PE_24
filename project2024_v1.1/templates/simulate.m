%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x,u,ctrl_info] = simulate(x0, ctrl, params)
    nx = params.model.nx;
    nu = params.model.nu;
    Nsim = params.exercise.SimHorizon;
    

    x = zeros(nx,Nsim+1);
    u = zeros(nu, Nsim);
    % This was the code written for exercise 8, passes the tests,
    % used in 27 again
    %ctrl_info = struct('ctrl_feas', cell(1,Nsim));
    % This is the adapted variant for exercise 22
    % ctrl_info = struct('ctrl_feas', cell(1,Nsim), 'objective', cell(1,Nsim), 'solvetime', cell(1,Nsim));

    x(:,1) = x0;

    for i= 1:Nsim
        % Get control input
        [u(:,i), ctrl_info(i)] = ctrl.eval(x(:,i));
        % Simulate system
        x(:,i+1) = params.model.A*x(:,i) + params.model.B*u(:,i);
    end
end