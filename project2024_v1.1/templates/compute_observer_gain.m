%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [L] = compute_observer_gain(poles, params_aug_obs)
    A = params_aug_obs.model.A;
    C = params_aug_obs.model.C;

    % Compute the observer gain
    
    L = place(A', -C', poles)';
    
    
    %{
    % Compute the matrix S
    S = A + L * C;

    % Calculate the poles of S (eigenvalues)
    poles_S = eig(S);

    % Display the poles of S
    disp('Desired poles:');
    disp(poles);
    disp('Poles of the matrix S:');
    disp(poles_S);
    %}
end