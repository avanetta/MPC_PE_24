%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC(Q,R,N,params)
            
            % ADD STUFF HERE
            % Get numerical data defining system
            A = params.model.A;
            B = params.model.B;
            nx = params.model.nx;
            nu = params.model.nu;
            
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;

            X0 = params.exercise.InitialConditionA;
            
            % Inifinite-horizon cost matrix P is also the solution to the
            % ARE
            P = zeros(nx,nx); %LQR infintite-horizon cost
            [K,P,S] = dlqr(A, B, Q, R);
            
            % Compute constraints and objective
            U = sdpvar(repmat(nu,1,N),repmat(1,1,N));

            constraints = [];
            objective = 0;
            x = X0;
            for k = 1:N
                x = A*x + B*U{k};
                objective = objective + x'*Q*x + U{k}'*R*U{k};
                % constraints = [constraints, x{k+1} == A*x{k} + B*U{k}];
                constraints = [constraints, H_u*U{k} <= h_u, H_x*x<=h_x];
            end

            %objective = objective + x'*P*x;


            % Already given except for comment{
            opts = sdpsettings('verbose',1,'solver','quadprog');
            % Standard form is optimizer(constraints, objective, sdpsettings, parameters_in, solutions_out
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{U{1} objective});
            % }
        end

        function [u, ctrl_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            ctrl_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end