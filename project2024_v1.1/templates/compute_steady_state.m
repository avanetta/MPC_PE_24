%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_s, u_s] = compute_steady_state(params, d)
        %{
        % does not work yet (errors with either singularities or not
        % matching dimensions
        
        % steady-state output C_ref * y(x_s,d) = T_ref
        % --> C_ref*( C*x_s + Cd*d ) = T_ref
        % --> C*x_s + Cd*d = inv(C_ref)*T_ref
        % --> x_s = inv(C)*( inv(C_ref)*T_ref - Cd*d )

        x_s = pinv(params.model.C)*(pinv(params.model.C_ref)*params.exercise.T_ref - params.model.Cd*d);
        % works

        % steady-state input x_s = A*x_s + B*u_s + Bd*d
        % --> (I-A)*x_s - Bd*d = B*u_s
        % --> u_s = inv(B)*( (I-A)*x_s - Bd*d )
        I = eye(params.model.nx);
        u_s = pinv(params.model.B)*((I-params.model.A)*x_s - params.model.Bd*d); %gives error for singularity and dimensions connected to SS-computation
        %}

        A = params.model.A;
        I = eye(params.model.nx);
        B = params.model.B;
        Cref = params.model.C_ref;
        C = params.model.C;
        ny = params.model.ny;
        nu = params.model.nu;
        Bd = params.model.Bd;
        Cd = params.model.Cd;
        Tref = params.exercise.T_ref;

        mat = [A-I, B;
                Cref*C, Cref*zeros(ny,nu)];
        vec = [-Bd*d; Tref-Cref*Cd*d];

        res = pinv(mat)*vec;
        x_s = res(1:params.model.nx);
        u_s = res(params.model.nx+1:end);
        assert(all(size(x_s) == [params.model.nx,1]));
        assert(all(size(u_s) == [params.model.nu,1]));

end
