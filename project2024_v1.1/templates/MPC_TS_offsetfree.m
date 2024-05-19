%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS_offsetfree
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS_offsetfree(Q,R,N,H,h,params)

            % ADD STUFF HERE
            A = params.model.A;
            B = params.model.B;
            B_d = params.model.Bd;
            nx = params.model.nx;
            nd = params.model.nd;
            nu = params.model.nu;
            
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;

            % Inifinite-horizon cost matrix P is also the solution to the ARE
            [K,P,S] = dlqr(A, B, Q, R); %P is matrix for LQR infintite-horizon cost

            % The optimization variables are U, X and D
            U = sdpvar(repmat(nu,1,N),repmat(1,1,N));
            X = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
            D = sdpvar(repmat(nd,1,N+1),repmat(1,1,N+1));
            XS = sdpvar(nx,1);
            US = sdpvar(nu,1);

            % Update constraints and obejctive function
            constraints = [];
            objective = 0;
            X0 = X{1};
            D0 = D{1};

            % "at each sampling time", means for each k (each step of the
            % horizon)
            for k = 1:N
                % Estimate state and disturbance x_est, d_est

                % Obtain (xs , us) from steady-state target problem using disturbance estimate
                % [x_s,u_s] = compute_steady_state(params,D{k});
                % it seems like params.model has no C_ref matrix as
                % element, which would be used inside of
                % compute_steady_state
                

                objective = objective + (X{k}-XS)'*Q*(X{k}-XS) + (U{k}-US)'*R*(U{k}-US);
                constraints = [constraints, X{k+1} == A*X{k} + B*U{k} + B_d*D{k}];
                constraints = [constraints, D{k+1} == D{k}];
                constraints = [constraints, H_u*U{k} <= h_u, H_x*X{k}<=h_x];

            end

            % Add constraint of x(N) and LQR IH cost
            constraints = [constraints, H*(X{N+1}-XS)<= h];
            objective = objective + (X{N+1}-XS)'*P*(X{N+1}-XS);

            % This was given
            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,{X0 D0 XS US},{U{1} objective});
        
        end

        function [u, ctrl_info] = eval(obj,x,d,x_s,u_s)
            % u = ...;
            % ctrl_info = .;
            %% evaluate control action by solving MPC problem, e.g.
            tic
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x, d, x_s, u_s);
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