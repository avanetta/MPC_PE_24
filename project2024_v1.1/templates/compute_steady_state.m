%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_s, u_s] = compute_steady_state(params, d)

        % does not work yet (errors with either singularities or not
        % matching dimensions
        
        % steady-state output C_ref * y(x_s,d) = T_ref
        % --> C_ref*( C*x_s + Cd*d ) = T_ref
        % --> C*x_s + Cd*d = inv(C_ref)*T_ref
        % --> x_s = inv(C)*( inv(C_ref)*T_ref - Cd*d )
        x_s = (params.exercise.T_ref\params.model.C_ref - params.model.Cd*d)\params.model.C;
        % works

        % steady-state input x_s = A*x_s + B*u_s + Bd*d
        % --> (I-A)*x_s - Bd*d = B*u_s
        % --> u_s = inv(B)*( (I-A)*x_s - Bd*d )
        I = eye(params.model.nx, params.model.nx);
        u_s = ((I-params.model.A)*x_s - params.model.Bd*d)\params.model.B; %gives error for singularity and dimensions connected to SS-computation
        
        %add comment to try git
        kjbpronerpoicjporicaporcpeaorcn
        
end
