%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ac, Bc, Bdc] = generate_system_cont_cc(params)

    model = params.model;
       
    Ac = [ -(model.a12+model.a1o)/model.m1           model.a12/model.m1                         0; ...
                model.a12/model.m2          -(model.a12+model.a23+model.a2o)/model.m2   model.a23/model.m2; ...
                        0                            model.a23/model.m3              -(model.a23+model.a3o)/model.m3];

    
    Bc = [1/model.m1        0; ...
            0             1/model.m2; ...
            0                 0];


    Bdc = [1/model.m1       0               0; ...
              0         1/model.m2          0; ...
              0             0           1/model.m3];
end