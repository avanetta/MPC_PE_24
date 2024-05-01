%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef LQR
    properties
        K
    end
    
    methods
        function obj = LQR(Q,R,params)
            % obj.K = ...
            % First Option: Batch Approach
            A = params.model.A;
            B = params.model.B;

            [K,~,~] = dlqr(A, B, Q, R);
            obj.K = K;
        end

        function [u, ctrl_info] = eval(obj,x)
            % u = ...
            % ctrl_info = ...
            u = -obj.K * x;
            ctrl_info.ctrl_feas = true;
        end
    end
end