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

            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            
            nx_c = size(H_x,1);
            nt = size(H,1);
            
            % Inifinite-horizon cost matrix P is also the solution to the ARE
            [K,P,S_LQR] = dlqr(A, B, Q, R); %P is matrix for LQR infintite-horizon cost
            
            % The optimization variables are U, X and E
            U = sdpvar(repmat(nu,1,N),repmat(1,1,N));
            X = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
            E = sdpvar(repmat(nx_c,1,N+1), repmat(1,1,N+1));
            E_t = sdpvar(nt,1,'full');


            % Update constraints and objective function
            constraints = [];
            objective = 0;
            X0 = X{1};
            for k = 1:N
                objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k};
                objective = objective + E{k}'*S*E{k} + v*norm(E{k},1);
                constraints = [constraints, X{k+1} == A*X{k} + B*U{k}];
                constraints = [constraints, H_x*X{k}<=h_x+E{k}];
                constraints = [constraints, H_u*U{k} <= h_u];
                constraints = [constraints, E{k} >= 0];
            end
            
            objective = objective + E_t' * S_t * E_t + v_t*norm(E_t,1);
            constraints = [constraints, H*X{N+1}<= h + E_t];
            constraints = [constraints, E_t >= 0];

            % Add IH LQR cost
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
