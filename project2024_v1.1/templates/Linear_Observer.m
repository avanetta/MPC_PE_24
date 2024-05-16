%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef Linear_Observer
    properties
        L % Observer gain matrix
        params
    end
    
    methods
        function obj = Linear_Observer(L, params)
            % Constructor to initialize the observer with the provided gain matrix L
            obj.L = L;
            obj.params = params;
        end
        
        function x0_est_pred = eval(obj, x0_est, u, y)
            A = obj.params.model.A;
            B = obj.params.model.B;
            C = obj.params.model.C;

            x0_est_pred = A*x0_est + B*u + obj.L*(C*x0_est - y);
        end
    end
end