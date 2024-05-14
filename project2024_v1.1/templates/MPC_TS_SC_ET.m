%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS_SC_ET
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS_SC_ET(Q, R, N, H, h, S, v, S_t, v_t, params)
            
            % ADD STUFF HERE
            % Get numerical data defining system
            A = params.model.A;
            B = params.model.B;
            nx = params.model.nx;
            nu = params.model.nu;

            nx_constr = size(S,1);
            nt = size(S_t,1);

            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            
            % Inifinite-horizon cost matrix P is also the solution to the ARE
            [K,P,S] = dlqr(A, B, Q, R); %P is matrix for LQR infintite-horizon cost
            
            % The optimization variables are U and X
            U = sdpvar(repmat(nu,1,N),repmat(1,1,N));
            X = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
            E = sdpvar(repmat(nx_constr,1,N+1),repmat(1,1,N+1));
            Et = sdpvar(repmat(nt,1,N+1),repmat(1,1,N+1));


            % Update constraints and obejctive function
            constraints = [];
            objective = 0;
            X0 = X{1};
            for k = 1:N
                objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k};
                constraints = [constraints, X{k+1} == A*X{k} + B*U{k}];
                constraints = [constraints, H_u*U{k} <= h_u, H_x*X{k}<=h_x];
            end

            for k = 1:N+1
                objective = objective + E{k}'*S*E{k} + v*norm(E{k},1);
                constraints = [constraints, H_u*U{k} <= h_u, H_x*X{k}<=h_x];
            end

            % FOR EX 21 ONLY
            % Add constraint of x(N) and LQR IH cost
            constraints = [constraints, H*X{N+1}<= h];
            objective = objective + X{N+1}'*P*X{N+1};
            
            opts = sdpsettings('verbose', 1, 'solver', 'quadprog', 'quadprog.TolFun', 1e-8);
            obj.yalmip_optimizer = optimizer(constraints, objective, opts, X0, {U{1} objective});
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

            ctrl_info = struct('ctrl_feas', feasible, 'objective', objective, 'solvetime', solvetime);
        end
    end
end
